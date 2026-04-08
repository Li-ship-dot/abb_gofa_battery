/**
 * @file hand_eye_calibrator.cpp
 * @brief 手眼标定节点 (Eye-to-Hand)
 *
 * 功能：计算机器人末端与相机之间的外参关系
 * 算法：使用 ArUco 标记进行手眼标定
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <string>
#include <filesystem>
#include <mutex>

using namespace std;

class HandEyeCalibrator : public rclcpp::Node
{
public:
    HandEyeCalibrator() : Node("hand_eye_calibrator")
    {
        this->declare_parameter("aruco_marker_length", 0.05);
        this->declare_parameter("min_samples", 10);
        this->declare_parameter("camera_frame", "camera_optical_frame");
        this->declare_parameter("camera_info_file", "config/d415_camera_info.yaml");
        this->declare_parameter("calibration_file", "");

        this->get_parameter("aruco_marker_length", marker_length_);
        this->get_parameter("min_samples", min_samples_);
        this->get_parameter("camera_frame", camera_frame_);
        this->get_parameter("calibration_file", calibration_file_);

        string camera_info_file;
        this->get_parameter("camera_info_file", camera_info_file);
        loadCameraInfo(camera_info_file);

        dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&HandEyeCalibrator::imageCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/egm/robot_pose", 10,
            std::bind(&HandEyeCalibrator::poseCallback, this, std::placeholders::_1));

        transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
            "/hand_eye_transform", 10);

        calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/calibrate",
            std::bind(&HandEyeCalibrator::calibrateCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/reset_samples",
            std::bind(&HandEyeCalibrator::resetCallback, this,
                     std::placeholders::_1, std::placeholders::_2));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        is_calibrated_ = false;
        has_valid_pose_ = false;
        debug_count_ = 0;
        last_pose_time_ = rclcpp::Time(0);

        // Timer for pose stale detection (10 second timeout)
        pose_stale_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&HandEyeCalibrator::checkPoseStale, this));

        // Try to load existing calibration from file (if path is configured and file exists)
        if (!calibration_file_.empty()) {
            loadCalibration();
        }

        RCLCPP_INFO(this->get_logger(), "Hand-Eye Calibrator initialized");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            vector<int> marker_ids;
            vector<vector<cv::Point2f>> marker_corners;
            cv::aruco::detectMarkers(frame, dict_, marker_corners, marker_ids);

            if (marker_ids.empty()) {
                if (debug_count_ % 30 == 0) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                        "No ArUco markers detected...");
                }
                debug_count_++;
                return;
            }

            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            // 估计每个标记的位姿
            vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, marker_length_,
                                                 camera_matrix_, dist_coeffs_,
                                                 rvecs, tvecs);

            // 绘制坐标轴
            for (size_t i = 0; i < rvecs.size(); i++) {
                cv::drawFrameAxes(frame, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.05);
            }

            #ifdef DEBUG_DISPLAY
            cv::imshow("Hand-Eye Calibration", frame);
            cv::waitKey(1);
#endif

            // 保存样本 - 使用第一个标记 (protected by mutex)
            bool valid;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                valid = has_valid_pose_;
            }
            if (valid && !rvecs.empty()) {
                saveSample(rvecs[0], tvecs[0]);
            }

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_robot_pose_ = *pose;
        has_valid_pose_ = true;
        last_pose_time_ = this->get_clock()->now();
    }

    void checkPoseStale()
    {
        bool valid;
        rclcpp::Time last_time;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            valid = has_valid_pose_;
            last_time = last_pose_time_;
        }

        if (!valid || last_time.seconds() == 0.0) {
            return;
        }
        double elapsed = (this->get_clock()->now() - last_time).seconds();
        if (elapsed > 10.0) {
            RCLCPP_WARN(this->get_logger(), "Robot pose stale (%.1f s since last update) - resetting has_valid_pose_", elapsed);
            std::lock_guard<std::mutex> lock(data_mutex_);
            has_valid_pose_ = false;
        }
    }

    void saveSample(const cv::Vec3d& rvec, const cv::Vec3d& tvec)
    {
        geometry_msgs::msg::Pose robot_pose;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            robot_pose = last_robot_pose_;
        }

        Sample sample;
        sample.robot_pose = robot_pose;
        sample.rvec = rvec;
        sample.tvec = tvec;

        size_t sample_count;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            samples_.push_back(sample);
            sample_count = samples_.size();
        }

        RCLCPP_INFO(this->get_logger(), "Sample %zu saved. Robot: [%.3f, %.3f, %.3f]",
                   sample_count,
                   robot_pose.position.x,
                   robot_pose.position.y,
                   robot_pose.position.z);

        if (sample_count >= (size_t)min_samples_ && !is_calibrated_) {
            RCLCPP_INFO(this->get_logger(), "Enough samples. Call /calibrate to compute transform.");
        }
    }

    void calibrateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        size_t current_size;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_size = samples_.size();
        }
        if (current_size < (size_t)min_samples_) {
            response->success = false;
            response->message = "Need at least " + std::to_string(min_samples_) + " samples";
            return;
        }

        if (is_calibrated_) {
            response->success = true;
            response->message = transform_to_string();
            return;
        }

        vector<Sample> samples_copy;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            RCLCPP_INFO(this->get_logger(), "Computing hand-eye transform with %zu samples...", samples_.size());
            samples_copy = samples_;  // Copy under lock to avoid iterator invalidation
        }

        vector<cv::Mat> R_gripper2base, T_gripper2base;
        vector<cv::Mat> R_target2cam, T_target2cam;

        for (const auto& sample : samples_copy) {
            cv::Mat T_rb = poseToMatrix(sample.robot_pose);
            cv::Mat R_rb = T_rb.rowRange(0, 3).colRange(0, 3).clone();
            cv::Mat t_rb = T_rb.rowRange(0, 3).col(3).clone();

            cv::Mat R_tc;
            cv::Rodrigues(sample.rvec, R_tc);
            cv::Mat T_tc = cv::Mat::eye(4, 4, CV_64F);
            R_tc.copyTo(T_tc.rowRange(0, 3).colRange(0, 3));
            T_tc.at<double>(0, 3) = sample.tvec[0];
            T_tc.at<double>(1, 3) = sample.tvec[1];
            T_tc.at<double>(2, 3) = sample.tvec[2];

            R_gripper2base.push_back(R_rb);
            T_gripper2base.push_back(t_rb);
            R_target2cam.push_back(R_tc.rowRange(0, 3).colRange(0, 3).clone());
            T_target2cam.push_back(T_tc.rowRange(0, 3).col(3).clone());
        }

        cv::Mat R_cam2gripper, T_cam2gripper;

        try {
            cv::calibrateHandEye(R_gripper2base, T_gripper2base,
                                R_target2cam, T_target2cam,
                                R_cam2gripper, T_cam2gripper,
                                cv::CALIB_HAND_EYE_TSAI);

            // 计算相机在基座下的位姿 (使用四元数平均避免SO(3)直接平均的病态问题)
            cv::Mat R_cam2base, T_cam2base;
            cv::Mat T_avg = cv::Mat::zeros(3, 1, CV_64F);

            // 四元数平均：先将所有旋转矩阵转换为四元数
            std::vector<tf2::Quaternion> quats;
            for (size_t i = 0; i < R_gripper2base.size(); i++) {
                // cv::Mat stores data in row-major order
                const double* m = R_gripper2base[i].ptr<double>();
                tf2::Matrix3x3 mat(m[0], m[1], m[2],   // row 0
                                   m[3], m[4], m[5],   // row 1
                                   m[6], m[7], m[8]);  // row 2
                tf2::Quaternion q;
                mat.getRotation(q);
                // 确保所有四元数指向同一半球(避免180度翻转)
                if (!quats.empty() && quats.back().dot(q) < 0) {
                    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
                }
                quats.push_back(q);
                T_avg += T_gripper2base[i];
            }
            T_avg /= (double)R_gripper2base.size();

            // 计算平均四元数 (算术平均后归一化)
            double w_sum = 0, x_sum = 0, y_sum = 0, z_sum = 0;
            for (const auto& q : quats) { w_sum += q.w(); x_sum += q.x(); y_sum += q.y(); z_sum += q.z(); }
            double norm = std::sqrt(w_sum*w_sum + x_sum*x_sum + y_sum*y_sum + z_sum*z_sum);
            if (norm < 1e-6) {
                throw std::runtime_error("Invalid rotation data: all zeros");
            }
            tf2::Quaternion q_avg(w_sum/norm, x_sum/norm, y_sum/norm, z_sum/norm);
            q_avg.normalize();

            // 将四元数转换为旋转矩阵 (使用标准公式)
            // q = (w, x, y, z), R = [1-2(y²+z²)  2(xy-wz)   2(xz+wy); ...]
            double w = q_avg.w(), x = q_avg.x(), y = q_avg.y(), z = q_avg.z();
            cv::Mat R_avg(3, 3, CV_64F);
            R_avg.at<double>(0, 0) = 1 - 2*(y*y + z*z);
            R_avg.at<double>(0, 1) = 2*(x*y - w*z);
            R_avg.at<double>(0, 2) = 2*(x*z + w*y);
            R_avg.at<double>(1, 0) = 2*(x*y + w*z);
            R_avg.at<double>(1, 1) = 1 - 2*(x*x + z*z);
            R_avg.at<double>(1, 2) = 2*(y*z - w*x);
            R_avg.at<double>(2, 0) = 2*(x*z - w*y);
            R_avg.at<double>(2, 1) = 2*(y*z + w*x);
            R_avg.at<double>(2, 2) = 1 - 2*(x*x + y*y);

            R_cam2base = R_avg * R_cam2gripper;
            T_cam2base = R_avg * T_cam2gripper + T_avg;

            cv::Rodrigues(R_cam2base, rvec_result_);
            tvec_result_ = T_cam2base.clone();

            publishTransform(rvec_result_, tvec_result_);
            is_calibrated_ = true;

            // Persist calibration to file if configured
            if (!calibration_file_.empty()) {
                if (saveCalibration()) {
                    RCLCPP_INFO(this->get_logger(), "Calibration saved to %s", calibration_file_.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to save calibration to %s", calibration_file_.c_str());
                }
            }

            response->success = true;
            response->message = "Calibration successful!\n" + transform_to_string();

            RCLCPP_INFO(this->get_logger(), "Hand-eye calibration successful!");

        } catch (const cv::Exception& e) {
            response->success = false;
            response->message = "Calibration failed: " + string(e.what());
            RCLCPP_ERROR(this->get_logger(), "Calibration error: %s", e.what());
        }
    }

    void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            samples_.clear();
        }
        is_calibrated_ = false;
        response->success = true;
        response->message = "Samples reset.";
        RCLCPP_INFO(this->get_logger(), "Samples reset.");
    }

    cv::Mat poseToMatrix(const geometry_msgs::msg::Pose& pose)
    {
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        T.at<double>(0, 3) = pose.position.x;
        T.at<double>(1, 3) = pose.position.y;
        T.at<double>(2, 3) = pose.position.z;

        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 R(q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T.at<double>(i, j) = R[i][j];
            }
        }
        return T;
    }

    void publishTransform(const cv::Vec3d& rvec, const cv::Mat& tvec)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = camera_frame_;

        transform.transform.translation.x = tvec.at<double>(0);
        transform.transform.translation.y = tvec.at<double>(1);
        transform.transform.translation.z = tvec.at<double>(2);

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        tf2::Matrix3x3 tf2_R;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                tf2_R[i][j] = R.at<double>(i, j);
            }
        }
        tf2::Quaternion q;
        tf2_R.getRotation(q);

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    string transform_to_string()
    {
        if (!is_calibrated_) {
            return "Not calibrated yet. Collect samples and call /calibrate.";
        }
        stringstream ss;
        ss << "Camera frame: " << camera_frame_ << "\n";
        ss << "Translation: [" << tvec_result_.at<double>(0) << ", "
           << tvec_result_.at<double>(1) << ", " << tvec_result_.at<double>(2) << "] m\n";
        ss << "Rotation (Rodrigues): [" << rvec_result_[0] << ", "
           << rvec_result_[1] << ", " << rvec_result_[2] << "]";
        return ss.str();
    }

    void loadCameraInfo(const string& filename)
    {
        // Try multiple paths: absolute, relative to ws/src, relative to current dir
        vector<string> paths = {
            filename,
            filesystem::path(filename).is_absolute() ? filename : "",
        };

        string resolved_path;
        for (const auto& p : paths) {
            if (p.empty()) continue;
            if (filesystem::exists(p)) {
                resolved_path = p;
                break;
            }
        }

        if (resolved_path.empty()) {
            // Fallback: try as package-relative path
            resolved_path = filename;
        }

        cv::FileStorage fs;
        try {
            fs.open(resolved_path, cv::FileStorage::READ);
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not parse camera info file '%s' (%s), using defaults",
                        resolved_path.c_str(), e.what());
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
            return;
        }
        if (!fs.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Could not open camera info file '%s', using defaults",
                        resolved_path.c_str());
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
            return;
        }

        cv::FileNode cm = fs["camera_matrix"];
        if (!cm.empty() && !cm.isNone()) {
            cm["data"] >> camera_matrix_;
        } else {
            RCLCPP_WARN(this->get_logger(), "camera_matrix not found in YAML, using D415 factory default");
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 385.5, 0.0, 319.5, 0.0, 385.5, 239.5, 0.0, 0.0, 1.0);
        }

        cv::FileNode dc = fs["distortion_coefficients"];
        if (!dc.empty() && !dc.isNone()) {
            dc["data"] >> dist_coeffs_;
        } else {
            RCLCPP_WARN(this->get_logger(), "distortion_coefficients not found in YAML, using zero distortion");
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        }

        fs.release();

        RCLCPP_INFO(this->get_logger(), "Loaded camera intrinsics from %s", resolved_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera matrix: [%g, %g, %g; %g, %g, %g; %g, %g, %g]",
                   camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(0,1), camera_matrix_.at<double>(0,2),
                   camera_matrix_.at<double>(1,0), camera_matrix_.at<double>(1,1), camera_matrix_.at<double>(1,2),
                   camera_matrix_.at<double>(2,0), camera_matrix_.at<double>(2,1), camera_matrix_.at<double>(2,2));
    }

    /**
     * @brief Load calibration from YAML file (if exists)
     * @return true if loaded successfully, false otherwise
     */
    bool loadCalibration()
    {
        if (calibration_file_.empty()) {
            return false;
        }
        std::filesystem::path p(calibration_file_);
        if (!std::filesystem::exists(p)) {
            RCLCPP_INFO(this->get_logger(), "Calibration file '%s' not found, will calibrate from scratch",
                       calibration_file_.c_str());
            return false;
        }

        cv::FileStorage fs(calibration_file_, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Could not open calibration file '%s'", calibration_file_.c_str());
            return false;
        }

        try {
            cv::FileNode tn = fs["transform"];
            tn["rvec"] >> rvec_result_;
            tn["tvec"] >> tvec_result_;

            string loaded_camera_frame;
            tn["camera_frame"] >> loaded_camera_frame;
            if (!loaded_camera_frame.empty() && loaded_camera_frame != camera_frame_) {
                RCLCPP_WARN(this->get_logger(), "Camera frame in file '%s' differs from parameter '%s', using parameter",
                           loaded_camera_frame.c_str(), camera_frame_.c_str());
            }

            fs.release();

            // Publish the loaded transform and set calibrated state
            publishTransform(rvec_result_, tvec_result_);
            is_calibrated_ = true;

            RCLCPP_INFO(this->get_logger(), "Loaded calibration from %s: [%+.4f, %+.4f, %+.4f] m",
                       calibration_file_.c_str(),
                       tvec_result_.at<double>(0), tvec_result_.at<double>(1), tvec_result_.at<double>(2));
            RCLCPP_INFO(this->get_logger(), "Loaded calibration: %s", transform_to_string().c_str());
            return true;
        } catch (const cv::Exception& e) {
            fs.release();
            RCLCPP_WARN(this->get_logger(), "Failed to parse calibration file '%s': %s",
                       calibration_file_.c_str(), e.what());
            return false;
        }
    }

    /**
     * @brief Save calibration to YAML file
     * @return true if saved successfully, false otherwise
     */
    bool saveCalibration() const
    {
        if (calibration_file_.empty()) {
            return false;
        }

        cv::FileStorage fs(calibration_file_, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open calibration file '%s' for writing",
                        calibration_file_.c_str());
            return false;
        }

        fs << "calibration_time" << rclcpp::Clock().now().seconds();
        fs << "camera_frame" << camera_frame_;
        fs << "transform" << "{";
        fs << "rvec" << rvec_result_;
        fs << "tvec" << tvec_result_;
        fs << "}";

        // Also save as rotation matrix and quaternion for readability
        cv::Mat R;
        cv::Rodrigues(rvec_result_, R);
        tf2::Matrix3x3 tf2_R(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        tf2::Quaternion q;
        tf2_R.getRotation(q);
        fs << "rotation_matrix" << R;
        fs << "quaternion" << "{";
        fs << "w" << q.w() << "x" << q.x() << "y" << q.y() << "z" << q.z();
        fs << "}";
        fs << "translation_m" << "{";
        fs << "x" << tvec_result_.at<double>(0);
        fs << "y" << tvec_result_.at<double>(1);
        fs << "z" << tvec_result_.at<double>(2);
        fs << "}";

        fs.release();
        RCLCPP_INFO(this->get_logger(), "Calibration saved to %s", calibration_file_.c_str());
        return true;
    }

    // 订阅/发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr pose_stale_timer_;

    // ArUco
    cv::Ptr<cv::aruco::Dictionary> dict_;
    double marker_length_;

    // 相机内参 (从 YAML 加载或默认)
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    int min_samples_;
    string camera_frame_;
    string calibration_file_;
    bool is_calibrated_;
    bool has_valid_pose_;
    int debug_count_;
    rclcpp::Time last_pose_time_;
    geometry_msgs::msg::Pose last_robot_pose_;
    std::mutex data_mutex_;

    struct Sample {
        geometry_msgs::msg::Pose robot_pose;
        cv::Vec3d rvec;
        cv::Vec3d tvec;
    };
    vector<Sample> samples_;

    cv::Vec3d rvec_result_;
    cv::Mat tvec_result_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HandEyeCalibrator>());
    rclcpp::shutdown();
    return 0;
}

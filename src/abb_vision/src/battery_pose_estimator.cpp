/**
 * @file battery_pose_estimator.cpp
 * @brief 电池3D位姿估计节点
 *
 * 使用PnP算法从4个角点像素坐标反算电池在机器人坐标系下的3D位姿
 *
 * 算法流程：
 * 1. 订阅 /battery_bboxes (4角点像素坐标)
 * 2. 从TF树获取 base_link → camera_optical_frame 变换
 * 3. 用cv::solvePnP求取camera_optical_frame下的电池pose
 * 4. 变换到base_link坐标系并发布
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <string>
#include <filesystem>
#include <cmath>
#include <memory>

using namespace std;

class BatteryPoseEstimator : public rclcpp::Node
{
public:
    BatteryPoseEstimator() : Node("battery_pose_estimator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // 声明参数
        this->declare_parameter("camera_info_file", "config/d415_camera_info.yaml");
        this->declare_parameter("battery_length_mm", 70.0);
        this->declare_parameter("battery_width_mm", 25.0);
        this->declare_parameter("camera_frame", "camera_optical_frame");
        this->declare_parameter("base_frame", "base_link");

        this->get_parameter("camera_info_file", camera_info_file_);
        this->get_parameter("battery_length_mm", battery_length_mm_);
        this->get_parameter("battery_width_mm", battery_width_mm_);
        this->get_parameter("camera_frame", camera_frame_);
        this->get_parameter("base_frame", base_frame_);

        // 加载相机内参
        loadCameraInfo(camera_info_file_);

        // 计算半尺寸（用于构建3D角点）
        half_length_ = battery_length_mm_ / 2.0;
        half_width_ = battery_width_mm_ / 2.0;

        RCLCPP_INFO(this->get_logger(), "Battery dimensions: %.1f x %.1f mm",
                    battery_length_mm_, battery_width_mm_);
        RCLCPP_INFO(this->get_logger(), "Half dimensions: L=%.1f, W=%.1f mm",
                    half_length_, half_width_);

        // 订阅电池边界框（4角点像素坐标）
        bbox_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/battery_bboxes", 10,
            std::bind(&BatteryPoseEstimator::bboxCallback, this, std::placeholders::_1));

        // 发布电池3D位姿
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/battery_poses", 10);

        // 等待TF就绪
        rclcpp::Rate loop_rate(10);
        bool tf_ready = false;
        for (int i = 0; i < 50 && !tf_ready; i++) {
            try {
                tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePoint(),
                                           tf2::durationFromSec(0.1));
                tf_ready = true;
                RCLCPP_INFO(this->get_logger(), "TF %s -> %s is available",
                            base_frame_.c_str(), camera_frame_.c_str());
            } catch (const tf2::TransformException& ex) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Waiting for TF: %s", ex.what());
                loop_rate.sleep();
            }
        }

        if (!tf_ready) {
            RCLCPP_WARN(this->get_logger(), "TF not available yet, will retry at runtime");
        }

        RCLCPP_INFO(this->get_logger(), "Battery Pose Estimator initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /battery_bboxes, publishing to /battery_poses");
    }

private:
    /**
     * @brief 边界框回调函数
     *
     * /battery_bboxes 格式：每4个pose为一个电池的角点
     * 每个pose.position包含(u, v, angle)
     */
    void bboxCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            return;
        }

        // 检查是否有有效的TF变换
        geometry_msgs::msg::TransformStamped T_base_cam;
        try {
            T_base_cam = tf_buffer_.lookupTransform(base_frame_, camera_frame_, tf2::TimePoint(),
                                                    tf2::durationFromSec(0.5));
            // TF成功，重置计数器
            if (tf_failure_count_ > 0) {
                RCLCPP_INFO(this->get_logger(), "TF lookup recovered after %d failures", tf_failure_count_);
                tf_failure_count_ = 0;
            }
        } catch (const tf2::TransformException& ex) {
            tf_failure_count_++;
            if (tf_failure_count_ > 10) {
                RCLCPP_ERROR(this->get_logger(), "TF lookup failed (count: %d): %s - too many failures, consider resetting",
                           tf_failure_count_, ex.what());
                tf_failure_count_ = 0;  // 重置计数器避免无限ERROR
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "TF lookup failed (count: %d): %s", tf_failure_count_, ex.what());
            }
            return;
        }

        // 计算TF矩阵 T_base_cam
        cv::Mat T_base_cam_mat = transformStampedToMat(T_base_cam);

        // 每4个pose为一组（一个电池的4个角点）
        size_t num_complete = msg->poses.size() / 4;
        size_t remainder = msg->poses.size() % 4;
        if (remainder != 0) {
            RCLCPP_WARN(this->get_logger(),
                        "Partial battery detection: %zu complete batteries, %zu leftover poses (%.1f%% discarded)",
                        num_complete, remainder,
                        100.0 * remainder / msg->poses.size());
        }
        if (num_complete == 0) {
            return;
        }

        geometry_msgs::msg::PoseArray output_msg;
        output_msg.header = msg->header;
        output_msg.header.frame_id = base_frame_;

        for (size_t i = 0; i < num_complete; i++) {
            // 提取4个角点的2D像素坐标
            vector<cv::Point2f> image_points;
            float angle = 0.0f;

            for (size_t j = 0; j < 4; j++) {
                size_t idx = i * 4 + j;
                image_points.push_back(cv::Point2f(
                    static_cast<float>(msg->poses[idx].position.x),
                    static_cast<float>(msg->poses[idx].position.y)));
                angle = static_cast<float>(msg->poses[idx].position.z);  // 角度在z分量
            }

            // 使用PnP估计位姿
            cv::Vec3d rvec, tvec;
            bool pnp_success = solvePnPForBattery(image_points, angle, rvec, tvec);

            if (!pnp_success) {
                RCLCPP_DEBUG(this->get_logger(), "PnP failed for battery %zu", i);
                continue;
            }

            // 将位姿从相机坐标系变换到base_link坐标系
            cv::Mat R_cam, t_cam;
            cv::Rodrigues(rvec, R_cam);

            // 构建电池在相机坐标下的位姿矩阵 T_bat_cam
            cv::Mat T_bat_cam = cv::Mat::eye(4, 4, CV_64F);
            R_cam.copyTo(T_bat_cam.rowRange(0, 3).colRange(0, 3));
            T_bat_cam.at<double>(0, 3) = tvec[0];
            T_bat_cam.at<double>(1, 3) = tvec[1];
            T_bat_cam.at<double>(2, 3) = tvec[2];

            // T_bat_base = T_base_cam * T_bat_cam
            cv::Mat T_bat_base = T_base_cam_mat * T_bat_cam;

            // 提取电池在base_link下的位置和姿态
            cv::Mat R_base = T_bat_base.rowRange(0, 3).colRange(0, 3).clone();
            cv::Mat t_base = T_bat_base.rowRange(0, 3).col(3).clone();

            // 转换旋转向量为四元数
            cv::Mat R_base_rod;
            cv::Rodrigues(R_base, R_base_rod);
            tf2::Matrix3x3 tf2_R(R_base.at<double>(0, 0), R_base.at<double>(0, 1), R_base.at<double>(0, 2),
                                R_base.at<double>(1, 0), R_base.at<double>(1, 1), R_base.at<double>(1, 2),
                                R_base.at<double>(2, 0), R_base.at<double>(2, 1), R_base.at<double>(2, 2));
            tf2::Quaternion q;
            tf2_R.getRotation(q);

            geometry_msgs::msg::Pose pose;
            pose.position.x = t_base.at<double>(0);
            pose.position.y = t_base.at<double>(1);
            pose.position.z = t_base.at<double>(2);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            output_msg.poses.push_back(pose);

            RCLCPP_DEBUG(this->get_logger(),
                        "Battery %zu: pos=[%.3f, %.3f, %.3f] m",
                        i, pose.position.x, pose.position.y, pose.position.z);
        }

        if (!output_msg.poses.empty()) {
            pose_pub_->publish(output_msg);
        }
    }

    /**
     * @brief 使用PnP求解电池在相机坐标系下的位姿
     *
     * 角点匹配策略：
     * 由于RotatedRect的角点顺序不确定，需要根据旋转向量一致性判断
     * 备选方案：用center + angle估计初始pose，再用PnP精修
     */
    bool solvePnPForBattery(const vector<cv::Point2f>& image_points, float angle,
                           cv::Vec3d& rvec, cv::Vec3d& tvec)
    {
        if (image_points.size() != 4) {
            return false;
        }

        // 构建3D角点坐标（电池平面内，Z=0）
        // 约定顺序：左上、右上、右下、左下（相对于电池自身坐标系）
        vector<cv::Point3f> model_points;
        model_points.push_back(cv::Point3f(-half_length_, -half_width_, 0.0f));  // 左上
        model_points.push_back(cv::Point3f( half_length_, -half_width_, 0.0f));  // 右上
        model_points.push_back(cv::Point3f( half_length_,  half_width_, 0.0f));  // 右下
        model_points.push_back(cv::Point3f(-half_length_,  half_width_, 0.0f));  // 左下

        // 计算图像中心
        cv::Point2f center(0, 0);
        for (const auto& p : image_points) {
            center += p;
        }
        center *= 0.25f;

        // 计算角点到中心的距离，用于排序
        vector<pair<float, size_t>> distances;
        for (size_t i = 0; i < image_points.size(); i++) {
            float dist = norm(image_points[i] - center);
            distances.push_back({dist, i});
        }
        sort(distances.begin(), distances.end(),
             [](const auto& a, const auto& b) { return a.first < b.first; });

        // 按距离排序：0=最近（可能是中心偏移），1-4=四个角点
        // 取距离最远的4个作为角点（排除可能的中心点）
        vector<cv::Point2f> sorted_points;
        vector<cv::Point3f> sorted_model;
        for (size_t i = 0; i < 4; i++) {
            sorted_points.push_back(image_points[distances[i].second]);
        }

        // 根据旋转向量一致性确定正确匹配
        // 尝试多种可能的匹配，选择重投影误差最小的
        return solvePnPWithBestMatch(sorted_points, model_points, rvec, tvec);
    }

    /**
     * @brief 尝试多种角点匹配，返回重投影误差最小的结果
     */
    bool solvePnPWithBestMatch(const vector<cv::Point2f>& image_points,
                               const vector<cv::Point3f>& model_points,
                               cv::Vec3d& best_rvec, cv::Vec3d& best_tvec)
    {
        // 定义4种可能的角点排列（基于电池长宽比的约束）
        // 长边在前：(-L,-W), (L,-W), (L,W), (-L,W) - 4种旋转
        const int permutations[4][4] = {
            {0, 1, 2, 3},  // 0°旋转
            {1, 2, 3, 0},  // 90°旋转
            {2, 3, 0, 1},  // 180°旋转
            {3, 0, 1, 2}   // 270°旋转
        };

        double min_error = numeric_limits<double>::max();
        bool found_valid = false;

        for (int p = 0; p < 4; p++) {
            vector<cv::Point2f> matched_image;
            vector<cv::Point3f> matched_model;

            for (int j = 0; j < 4; j++) {
                matched_image.push_back(image_points[permutations[p][j]]);
                matched_model.push_back(model_points[j]);
            }

            // 执行PnP
            cv::Vec3d rvec, tvec;
            // 使用IPPE方法——专门为平面物体设计，比ITERATIVE更精确
            bool success = cv::solvePnP(matched_model, matched_image,
                                        camera_matrix_, dist_coeffs_,
                                        rvec, tvec,
                                        false,  // useExtrinsicGuess
                                        cv::SOLVEPNP_IPPE);

            if (!success) {
                continue;
            }

            // 计算重投影误差
            vector<cv::Point2f> reproj_points;
            cv::projectPoints(matched_model, rvec, tvec, camera_matrix_, dist_coeffs_, reproj_points);

            double error = 0.0;
            for (size_t j = 0; j < 4; j++) {
                error += norm(matched_image[j] - reproj_points[j]);
            }
            error /= 4.0;

            if (error < min_error) {
                min_error = error;
                best_rvec = rvec;
                best_tvec = tvec;
                found_valid = true;
            }
        }

        // T7 Fix: Tighten PnP error threshold from 50px to 10px to reject bad pose estimates
        if (found_valid && min_error > 10.0) {
            RCLCPP_DEBUG(this->get_logger(), "Best PnP error %.1f px exceeds threshold (10px), rejecting solution",
                        min_error);
            found_valid = false;
        }

        // T7 Fix: Add physics-based reasonableness check for Z coordinate
        // Battery should be at a reasonable height above base_link (typically 0.1-0.5m for tabletop scenarios)
        if (found_valid) {
            constexpr double MIN_Z = 0.02;   // 20mm minimum (battery on surface)
            constexpr double MAX_Z = 1.0;    // 1m maximum (safety bound)
            if (best_tvec[2] < MIN_Z || best_tvec[2] > MAX_Z) {
                RCLCPP_DEBUG(this->get_logger(), "PnP Z=%.3f outside reasonable range [%.3f, %.3f]m, rejecting",
                            best_tvec[2], MIN_Z, MAX_Z);
                found_valid = false;
            }
        }

        return found_valid;
    }

    /**
     * @brief 将geometry_msgs TransformStamped转换为cv::Mat 4x4
     */
    cv::Mat transformStampedToMat(const geometry_msgs::msg::TransformStamped& ts)
    {
        cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

        T.at<double>(0, 3) = ts.transform.translation.x;
        T.at<double>(1, 3) = ts.transform.translation.y;
        T.at<double>(2, 3) = ts.transform.translation.z;

        tf2::Quaternion q(ts.transform.rotation.x, ts.transform.rotation.y,
                          ts.transform.rotation.z, ts.transform.rotation.w);
        tf2::Matrix3x3 R(q);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T.at<double>(i, j) = R[i][j];
            }
        }

        return T;
    }

    /**
     * @brief 从YAML文件加载相机内参
     */
    void loadCameraInfo(const string& filename)
    {
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
            resolved_path = filename;
        }

        cv::FileStorage fs;
        try {
            fs.open(resolved_path, cv::FileStorage::READ);
        } catch (const cv::Exception& e) {
            RCLCPP_WARN(this->get_logger(),
                        "Could not parse camera info file '%s' (%s), using defaults",
                        resolved_path.c_str(), e.what());
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 385.0, 0.0, 320.0,
                                                     0.0, 385.0, 240.0,
                                                     0.0, 0.0, 1.0);
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
            return;
        }
        if (!fs.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Could not open camera info file '%s', using defaults",
                        resolved_path.c_str());
            // RealSense factory defaults (D415 at 640x480: fx≈385, D455 at 640x480: fx≈615)
            // NOTE: These are estimates only. Run camera_calibration for accurate values.
            camera_matrix_ = (cv::Mat_<double>(3, 3) << 385.0, 0.0, 320.0,
                                                     0.0, 385.0, 240.0,
                                                     0.0, 0.0, 1.0);
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

        // Warn if uncalibrated placeholder values are detected
        double fx = camera_matrix_.at<double>(0, 0);
        bool has_zero_distortion = cv::countNonZero(dist_coeffs_) == 0;
        if (fx > 600.0 || has_zero_distortion) {
            RCLCPP_WARN(this->get_logger(),
                        "Camera intrinsics appear uncalibrated (fx=%.1f, distortion_zeros=%d). "
                        "Replace d415_camera_info.yaml with calibrated values for accurate pose estimation.",
                        fx, has_zero_distortion ? 1 : 0);
        } else {
            RCLCPP_INFO(this->get_logger(), "Loaded camera intrinsics from %s", resolved_path.c_str());
        }
    }

    // 订阅/发布
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr bbox_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    int tf_failure_count_ = 0;

    // 参数
    string camera_info_file_;
    string camera_frame_;
    string base_frame_;
    double battery_length_mm_;
    double battery_width_mm_;
    double half_length_;
    double half_width_;

    // 相机内参
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}

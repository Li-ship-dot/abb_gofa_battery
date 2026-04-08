/*
 * joint_to_cartesian_pose_node.cpp
 *
 * Subscribes to /joint_states and publishes the TCP pose in base_link frame
 * to /egm/robot_pose for hand-eye calibration.
 *
 * Uses tf2_ros::Buffer to lookup the transform published by robot_state_publisher,
 * avoiding redundant forward kinematics calculation.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
// NOTE: PoseStamped removed - publisher uses geometry_msgs::msg::Pose
// to match subscriber expectations in hand_eye_calibrator and
// battery_alignment_controller>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <memory>
#include <string>

namespace abb_rws_client
{

class JointToCartesianPoseNode : public rclcpp::Node
{
public:
  JointToCartesianPoseNode()
  : Node("joint_to_cartesian_pose"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare parameters
    target_frame_ = this->declare_parameter<std::string>("target_frame", "tool0");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "base_link");
    source_topic_ = this->declare_parameter<std::string>("source_topic", "/joint_states");

    RCLCPP_INFO(this->get_logger(), "Joint to Cartesian Pose Node started");
    RCLCPP_INFO(this->get_logger(), "  source_frame: %s", source_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  target_frame: %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  source_topic: %s", source_topic_.c_str());

    // Subscription to joint states (triggers TF update via robot_state_publisher)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      source_topic_,
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Validate joint state content before triggering pose publication
        if (msg->name.size() < 6 || msg->position.size() < 6) {
          // Not enough joints yet - TF lookup would return stale/zero transform
          return;
        }
        // Joint states trigger robot_state_publisher to update TF tree.
        // We just need to lookup the transform after joint states are received.
        this->publish_pose();
      });

    // Publisher for Cartesian pose - use Pose (not PoseStamped) to match subscriber types
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/egm/robot_pose", 10);

  }

private:
  void publish_pose()
  {
    try
    {
      // Lookup transform from source_frame (base_link) to target_frame (tool0)
      // This transform is published by robot_state_publisher based on joint states
      // Use tf2::TimePoint() instead of TimePointZero for Humble compatibility
      auto transform = tf_buffer_.lookupTransform(
        source_frame_,
        target_frame_,
        tf2::TimePoint(),
        std::chrono::milliseconds(100));

      // Publish as Pose (not PoseStamped) to match subscriber expectations
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;

      pose_pub_->publish(pose);
    }
    catch (const tf2::TransformException & ex)
    {
      // Don't spam on first few failures - TF may not be available yet
      static std::atomic<int> failure_count = 0;
      int count = failure_count++;
      if (count <= 5)
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "TF lookup failed: %s (count: %d)",
          ex.what(),
          count);
      }
    }
  }

  // Parameters
  std::string target_frame_;
  std::string source_frame_;
  std::string source_topic_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
};

}  // namespace abb_rws_client

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<abb_rws_client::JointToCartesianPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
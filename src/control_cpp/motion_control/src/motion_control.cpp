#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class MotionTrackingNode : public rclcpp::Node {
  public:
    MotionTrackingNode() : Node("motion_tracking") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                                                       std::bind(&MotionTrackingNode::odomCallback, this,
                                                                                 std::placeholders::_1));

        cmd_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/cmd", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MotionTrackingNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Motion Tracking node started.");
    }

  private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = *msg; }

    void controlLoop() {
        if (!latest_odom_.has_value())
            return;

        auto cmd = ackermann_msgs::msg::AckermannDriveStamped();
        cmd.header.stamp = this->get_clock()->now();
        cmd.drive.speed = 1.0;           // Example: constant forward speed
        cmd.drive.steering_angle = 0.1;  // Example: light turn
        cmd_pub_->publish(cmd);
    }

    std::optional<nav_msgs::msg::Odometry> latest_odom_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionTrackingNode>());
    rclcpp::shutdown();
    return 0;
}

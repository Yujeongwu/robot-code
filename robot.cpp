#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
  int step_;
  bool wall_following_;  // Flag to indicate whether the turtle is wall following

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0), wall_following_(false)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile, callback);
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::Twist vel;

    // Check if an obstacle is near (within 25cm)
    if (is_obstacle_near(scan, 0.25))
    {
      // Obstacle detected nearby, rotate to avoid obstacle
      vel.linear.x = 0.;
      vel.angular.z = 0.5;  // Rotate to avoid the obstacle
      wall_following_ = false;  // Stop wall following
    }
    else if (wall_following_)
    {
      // Wall following mode
      vel.linear.x = std::max(0.15, 0.1);  // Maintain a minimum linear speed of 15cm/s
      vel.angular.z = calculate_angular_velocity(scan, 35.0);  // Follow the wall
    }
    else
    {
      // Initial mode: Move forward until a wall is detected
      vel.linear.x = 0.2;
      vel.angular.z = 0.;
      if (is_wall_in_front(scan, 0.5))
      {
        // Wall detected in front, switch to wall following mode
        wall_following_ = true;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("self_drive"),
                "step=%d, range=%1.2f, linear=%1.2f, angular=%1.2f, wall_following=%d",
                step_, scan->ranges[0], vel.linear.x, vel.angular.z, static_cast<int>(wall_following_));

    pose_pub_->publish(vel);
    step_++;
  }

private:
  bool is_obstacle_near(const sensor_msgs::msg::LaserScan::SharedPtr scan, double threshold)
  {
    // Check if there is an obstacle nearby
    for (auto range : scan->ranges)
    {
      if (range < threshold)
      {
        return true;
      }
    }
    return false;
  }

  bool is_wall_in_front(const sensor_msgs::msg::LaserScan::SharedPtr scan, double threshold)
  {
    // Check if there is a wall in front of the turtle
    return scan->ranges[0] < threshold;
  }

  double calculate_angular_velocity(const sensor_msgs::msg::LaserScan::SharedPtr scan, double target_distance)
  {
    double wall_distance = scan->ranges[0];  // Distance to the wall in front of the turtle
    double error = wall_distance - target_distance;

    // P-controller for wall following
    double kp = 0.1;  // Proportional gain
    double angular_velocity = kp * error;

    return angular_velocity;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


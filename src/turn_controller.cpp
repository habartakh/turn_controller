#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

struct WayPoint {
  double dx;   // change in the x-coordinate of the robot's position
  double dy;   // change in the y-coordinate of the robot's position
  double dphi; // change in the orientation angle of the robot

  WayPoint(double a = 0.0, double b = 0.0, double c = 0.0)
      : dx(a), dy(b), dphi(c) {}
};

class TurnController : public rclcpp::Node {
public:
  TurnController() : Node("turn_controller") {

    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom_callback_group_;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TurnController::odom_callback, this, _1), options1);

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    twist_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    twist_timer = this->create_wall_timer(
        100ms, std::bind(&TurnController::control_loop, this),
        timer_callback_group_);

    // Setup the waypoints the robot passes through
    waypoints_traj_init();

    RCLCPP_INFO(this->get_logger(), "Initialized distance controller node");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Compute robot yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);
    // RCLCPP_INFO(this->get_logger(),
    //             "Received Odometry - current_yaw: %f radians", current_yaw);

    dphi += current_yaw - old_yaw;
    old_yaw = current_yaw;

    // RCLCPP_INFO(this->get_logger(), "Received Odometry - dphi: %f radians",
    //             dphi);
  }

  // Add all the waypoints the robot is going throughout the trajectory
  void waypoints_traj_init() {

    // waypoints_traj.push_back(WayPoint(0.000, 0.000, -0.844)); // yaw =
    // -0.843998 waypoints_traj.push_back(WayPoint(0.000, 0.000, +0.582)); //
    // yaw = -0.262086 waypoints_traj.push_back(WayPoint(0.000, 0.000, +0.329));
    // // yaw = -0.590681

    /*************** For test purposes ***********************/
    waypoints_traj.push_back(
        WayPoint(0.000, 0.000, +1.5708)); // For test purposes 90°
    waypoints_traj.push_back(
        WayPoint(0.000, 0.000, -1.5708)); // For test purposes 0°
    waypoints_traj.push_back(
        WayPoint(0.000, 0.000, -1.5708)); // For test purposes -90°
  }

  // Rotate the robot according to the desired yaw
  void control_loop() {

    RCLCPP_INFO(this->get_logger(), "dphi : %f", dphi);

    if (traj_index >= waypoints_traj.size()) {
      stop_robot();
      RCLCPP_INFO_ONCE(this->get_logger(), "Completed the trajectory! ");
      rclcpp::shutdown();
    }

    WayPoint target = waypoints_traj[traj_index];

    // Compute the distance left to the target
    double error_yaw = dphi - target.dphi;

    // If the robot reached the target waypoint
    if (std::abs(error_yaw) < 0.1) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint number : %lu",
                  traj_index + 1);

      // Stop the robot for 20 * 0.1 = 2 seconds

      stop_robot();

      traj_index++; // Update the next motion index

      // reset the yaw difference computed between the two waypoints
      dphi = 0.0;
      rclcpp::sleep_for(std::chrono::seconds(2));
    }

    // Calculate delta time
    rclcpp::Time now = this->now();
    double dt =
        prev_time.nanoseconds() == 0 ? 0.05 : (now - prev_time).seconds();
    prev_time = now;

    // Integral terms
    integral_yaw += error_yaw * dt;

    // Derivative terms
    double derivative_yaw = (error_yaw - prev_error_yaw) / dt;

    // PID control law
    double angular_speed =
        kp * error_yaw + ki * integral_yaw + kd * derivative_yaw;

    // Make sure the robot stays within max speed bounds
    angular_speed =
        std::clamp(angular_speed, -max_angular_speed, +max_angular_speed);

    RCLCPP_INFO(this->get_logger(), "angular_speed = %f ", angular_speed);

    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = angular_speed;

    twist_pub->publish(twist_cmd);

    prev_error_yaw = error_yaw;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.angular.z = 0.0;

    twist_pub->publish(stop_msg);
  }

  // Variable declarations
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  // Parameters used to compute the distance travelled
  double old_yaw = 0.0;
  double current_yaw = 0.0;
  double dphi = 0.0; // yaw difference between two waypoints

  // Waypoints the robot is passing by
  std::vector<WayPoint> waypoints_traj;
  long unsigned int traj_index = 0;

  // Parameters to move the robot
  geometry_msgs::msg::Twist twist_cmd;

  // PID Controller Parameters
  double kp = 0.5;           // Proportional Gain
  double ki = 0.0;           // Integral Gain
  double kd = 0.0;           // Derivative Gain
  double integral_yaw = 0.0; // Integral terms of the PID controller
  rclcpp::Time prev_time;    // instant t-1
  double prev_error_yaw = 0.0;
  double max_angular_speed =
      3.14; // source: https://husarion.com/manuals/rosbot-xl/
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<TurnController> turn_controller =
      std::make_shared<TurnController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(turn_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
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
    m.getRPY(roll, pitch, current_yaw); // yaw is in radians
    RCLCPP_INFO(this->get_logger(),
                "Received Odometry - current_yaw: %f radians", current_yaw);

    dphi += current_yaw - old_yaw;
    old_yaw = current_yaw;

    RCLCPP_INFO(this->get_logger(), "Received Odometry - dphi: %f radians",
                dphi);
  }

  // Add all the waypoints the robot is going throughout the trajectory
  void waypoints_traj_init() {

    waypoints_traj.push_back(WayPoint(0.000, 0.000, -0.844)); // yaw = -0.843998
    waypoints_traj.push_back(WayPoint(0.000, 0.000, +0.582)); // yaw = -0.262086
    waypoints_traj.push_back(WayPoint(0.000, 0.000, +0.329)); // yaw = -0.590681

/***************For test purposes ***********************/
    // waypoints_traj.push_back(
    //     WayPoint(0.000, 0.000, +1.57)); // For test purposes 90°
    // waypoints_traj.push_back(
    //     WayPoint(0.000, 0.000, -1.57)); // For test purposes 0°
    // waypoints_traj.push_back(
    //     WayPoint(0.000, 0.000, -1.57)); // For test purposes -90°
  }

  // Move the robot according to the desired trajectory
  void control_loop() {}

  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = 0.0;

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
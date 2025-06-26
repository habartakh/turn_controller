#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TurnController : public rclcpp::Node {
public:
  TurnController() : Node("turn_controller") {

    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom_callback_group_;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10,
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
    // Compute distance travelled in X and Y axis
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
  double dphi = 0.0; // yaw between two odom messages

  double turned_angle = 0.0; // how much the yaw increased

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
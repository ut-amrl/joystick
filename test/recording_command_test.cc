#include "joystick.h"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

int main(int argc, char** argv) {
  setenv("ROS_DOMAIN_ID", "184", 1);
  rclcpp::init(argc, argv);
  auto source = rclcpp::Node::make_shared("record_button_test");
  auto recorder = rclcpp::Node::make_shared("record_command_test");
  std::vector<bool> commands;
  auto subscription = recorder->create_subscription<std_msgs::msg::Bool>("/record", 10,
      [&](std_msgs::msg::Bool::ConstSharedPtr msg) { commands.push_back(msg->data); });
  auto feedback = recorder->create_publisher<std_msgs::msg::Int32>("/recording", 10);
  joystick::RecordingControl control(source);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(source);
  executor.add_node(recorder);
  auto until = [&](const std::function<bool()>& predicate) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
    do {
      executor.spin_some();
      if (predicate()) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("record command or feedback was not delivered");
  };
  until([&] { return subscription->get_publisher_count() > 0 && feedback->get_subscription_count() > 0; });
  const std::vector<int32_t> raw_buttons{1};
  control.Update(raw_buttons, 0, 0, 0);
  until([&] { return commands.size() == 1; });
  if (!commands[0] || raw_buttons[0] != 1) throw std::runtime_error("bad start command or mutated raw buttons");
  control.Update(raw_buttons, 0, 0, 0);  // held button must not repeat
  control.Update({0}, 0, 0, 0);
  control.Update(raw_buttons, 0, 0, 0);
  until([&] { return commands.size() == 2; });
  if (commands[1]) throw std::runtime_error("toggle did not send stop");
  std_msgs::msg::Int32 state;
  state.data = 1;  // a separate operator started a recording
  feedback->publish(state);
  for (int i = 0; i < 30; ++i) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  control.Update({0}, 0, 0, 0);
  control.Update(raw_buttons, 0, 0, 0);
  until([&] { return commands.size() == 3; });
  if (commands[2]) throw std::runtime_error("toggle ignored /recording feedback");
  rclcpp::shutdown();
}

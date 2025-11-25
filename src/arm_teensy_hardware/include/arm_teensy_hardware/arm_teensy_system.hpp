#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arm_teensy_hardware
{

class ArmTeensySystem : public hardware_interface::SystemInterface
{
public:
  // Lifecycle hooks
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Main read / write hooks
  hardware_interface::return_type
  read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string port_;
  int baudrate_{115200};

  // Latest commanded positions (one per joint)
  std::vector<double> last_commands_;

  // Serial handle (Teensy)
  int serial_fd_{-1};

  bool open_serial();
  void close_serial();
  bool send_command_packet(const std::vector<double> & positions);
  bool read_feedback_packet(std::vector<double> & positions);
};

}  // namespace arm_teensy_hardware

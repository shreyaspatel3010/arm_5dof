#include "arm_teensy_hardware/arm_teensy_system.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"  // HW_IF_POSITION etc.

namespace arm_teensy_hardware
{

// ============== Lifecycle ==============

hardware_interface::CallbackReturn
ArmTeensySystem::on_init(const hardware_interface::HardwareInfo & info)
{
  // Let the base class process generic info
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Parent on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    port_ = info.hardware_parameters.at("port");
    baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Missing or invalid hardware parameters (port / baudrate): %s",
      e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize command buffer with one entry per joint
  last_commands_.assign(info_.joints.size(), 0.0);

  RCLCPP_INFO(
    get_logger(),
    "ArmTeensySystem initialized for port %s @ %d",
    port_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmTeensySystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port '%s'", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Serial port %s opened", port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmTeensySystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "ArmTeensySystem activated");
  // You could send a "enable motors" command here if needed
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmTeensySystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "ArmTeensySystem deactivated");
  // You could send a "disable motors" command here if needed
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============== Read / write ==============

hardware_interface::return_type
ArmTeensySystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // If you don't have encoders yet, just mirror commands into states
  // using the framework-managed interfaces: <joint_name>/position

  // OPTIONAL: if you ever implement feedback, read it here
  std::vector<double> feedback;
  bool have_feedback = read_feedback_packet(feedback) &&
                       feedback.size() == info_.joints.size();

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;
    const std::string iface_name =
      joint_name + "/" + std::string(hardware_interface::HW_IF_POSITION);

    if (!has_state(iface_name))
    {
      // Shouldn't normally happen if your ros2_control tag is correct
      RCLCPP_WARN_ONCE(
        get_logger(),
        "State interface '%s' does not exist", iface_name.c_str());
      continue;
    }

    const double value = have_feedback ? feedback[i] : last_commands_[i];
    set_state(iface_name, value);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmTeensySystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read commands from the framework-managed command interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint_name = info_.joints[i].name;
    const std::string iface_name =
      joint_name + "/" + std::string(hardware_interface::HW_IF_POSITION);

    if (!has_command(iface_name))
    {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "Command interface '%s' does not exist", iface_name.c_str());
      continue;
    }

    // In Jazzy, get_command<T> returns T directly (no std::optional)
    double cmd = get_command(iface_name);
    last_commands_[i] = cmd;
  }

  if (!send_command_packet(last_commands_))
  {
    RCLCPP_ERROR(get_logger(), "Failed to send command packet to Teensy");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ============== Serial helpers ==============

bool ArmTeensySystem::open_serial()
{
  // Basic POSIX serial open – you can change to your favorite library
  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "open(%s) failed: %s",
      port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "tcgetattr() failed: %s",
      std::strerror(errno));
    close_serial();
    return false;
  }

  // Baud rate
  speed_t speed = B115200;
  // You can map baudrate_ to speed_t if you want different rates
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 5;

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "tcsetattr() failed: %s",
      std::strerror(errno));
    close_serial();
    return false;
  }

  return true;
}

void ArmTeensySystem::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArmTeensySystem::send_command_packet(const std::vector<double> & positions)
{
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(get_logger(), "Serial port not open");
    return false;
  }

  // Very simple text protocol: "q0:0.10,q1:-0.23,...\n"
  std::string line;
  for (size_t i = 0; i < positions.size(); ++i)
  {
    if (i > 0)
    {
      line += ",";
    }
    line += "q" + std::to_string(i) + ":" + std::to_string(positions[i]);
  }
  line += "\n";

  ssize_t n = ::write(serial_fd_, line.c_str(), line.size());
  if (n != static_cast<ssize_t>(line.size()))
  {
    RCLCPP_ERROR(
      get_logger(),
      "write() sent %zd of %zu bytes",
      n, line.size());
    return false;
  }

  return true;
}

bool ArmTeensySystem::read_feedback_packet(std::vector<double> &)
{
  // Optional: implement if you have encoders on the arm.
  // For now, just return false to indicate "no feedback".
  return false;
}

}  // namespace arm_teensy_hardware

// Export plugin
PLUGINLIB_EXPORT_CLASS(
  arm_teensy_hardware::ArmTeensySystem,
  hardware_interface::SystemInterface)

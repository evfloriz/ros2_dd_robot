#ifndef DD_HARDWARE_INTERFACE_HPP_
#define DD_HARDWARE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rpi_comms.hpp"

namespace dd_hardware_interface {

class DDHardwareInterface : public hardware_interface::SystemInterface {

public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    double l_wheel_vel;
    double l_wheel_pos;
    double r_wheel_vel;
    double r_wheel_pos;

    double l_wheel_cmd;
    double r_wheel_cmd;

    RPiComms rpicomms;
};

}

#endif

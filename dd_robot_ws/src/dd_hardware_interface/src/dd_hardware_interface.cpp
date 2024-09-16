// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <netdb.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <pigpiod_if2.h>

#include "dd_hardware_interface/dd_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dd_hardware_interface {

hardware_interface::CallbackReturn DDHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DDHardwareInterface"), "Initializing...");

    // Initialize rpicomms
    if (rpicomms.init() < 0) {
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DDHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &l_wheel_vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &l_wheel_pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &r_wheel_vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &r_wheel_pos));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DDHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &l_wheel_cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &r_wheel_cmd));

    return command_interfaces;
}

hardware_interface::CallbackReturn DDHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Shutdown rpicomms
    rpicomms.shutdown();

    RCLCPP_INFO(rclcpp::get_logger("DDHardwareInterface"), "Shutting down...");

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DDHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    rpicomms.read(l_wheel_vel, l_wheel_pos, r_wheel_vel, r_wheel_pos);

    //RCLCPP_INFO(rclcpp::get_logger("DDHardwareInterface"), "Got position state %.5f and velocity state %.5f for '%s'!", l_wheel_pos, l_wheel_vel, info_.joints[0].name.c_str());
    //RCLCPP_INFO(rclcpp::get_logger("DDHardwareInterface"), "Got position state %.5f and velocity state %.5f for '%s'!", r_wheel_pos, r_wheel_vel, info_.joints[1].name.c_str());
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DDHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    rpicomms.write(l_wheel_cmd, r_wheel_cmd);

    //RCLCPP_INFO(rclcpp::get_logger("DDHardwareInterface"), "Got command %.5f for '%s'!", l_wheel_cmd, info_.joints[0].name.c_str());
    //RCLCPP_INFO(rclcpp::get_logger("DDHardwareInterface"), "Got command %.5f for '%s'!", r_wheel_cmd, info_.joints[1].name.c_str());
    
    return hardware_interface::return_type::OK;
}

}  // namespace dd_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dd_hardware_interface::DDHardwareInterface, hardware_interface::SystemInterface)

#include "dd_hardware_interface/rpi_comms.hpp"

RPiComms::RPiComms() {
}

int RPiComms::read(double& l_wheel_vel_, double& l_wheel_pos_, double& r_wheel_vel_, double& r_wheel_pos_) {
    l_wheel_vel = l_wheel_vel_;
    l_wheel_pos = l_wheel_pos_;
    r_wheel_vel = r_wheel_vel_;
    r_wheel_pos = r_wheel_pos_;
    return 0;
}

int RPiComms::write(double& l_wheel_cmd, double& r_wheel_cmd) {
    return 0;
}

#ifndef RPI_COMMS_HPP_
#define RPI_COMMS_HPP_

class RPiComms {

public:
    RPiComms();

    int read(double& l_wheel_vel_, double& l_wheel_pos_, double& r_wheel_vel_, double& r_wheel_pos_);
    int write(double& l_wheel_cmd, double& r_wheel_cmd);

private:
    double l_wheel_vel;
    double l_wheel_pos;
    double r_wheel_vel;
    double r_wheel_pos;

    double l_wheel_cmd;
    double r_wheel_cmd;

};

#endif

#ifndef RPI_COMMS_HPP_
#define RPI_COMMS_HPP_

#include <map>
#include <string>
#include <stdint.h>
#include <math.h>

class RPiComms {

public:
    RPiComms();

    int init();
    int shutdown();
    int read(double& l_wheel_vel, double& l_wheel_pos, double& r_wheel_vel, double& r_wheel_pos);
    int write(double& l_wheel_cmd, double& r_wheel_cmd);

    static void callback_wrapper(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void* userdata);
    //void callback(int pi, unsigned user_gpio, unsigned level, uint32_t tick);

private:
    double l_wheel_vel_;
    double l_wheel_pos_;
    double r_wheel_vel_;
    double r_wheel_pos_;

    double l_wheel_cmd_;
    double r_wheel_cmd_;

    int gpioHandle;

    std::map<std::string, int> pins = {
        {"PWMA", 12},
        {"AIN2", 16},
        {"AIN1", 20},
        {"STBY", 21},
        {"BIN1", 19},
        {"BIN2", 26},
        {"PWMB", 13},
        
        {"ENCA", 22},
        {"ENCB", 23},
    };

    std::chrono::time_point<std::chrono::system_clock> time_;

    double ticks_per_rev = 40;
    double rads_per_tick = (2*M_PI) / ticks_per_rev;
    
    struct Encoder {
        int value = 0;
        int direction = 0;
    };
    Encoder encA_;
    Encoder encB_;
};

#endif

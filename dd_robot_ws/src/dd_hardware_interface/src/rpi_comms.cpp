#include <chrono>
#include <pigpiod_if2.h>

#include "dd_hardware_interface/rpi_comms.hpp"

RPiComms::RPiComms() {
}

int RPiComms::init() {
    gpioHandle = pigpio_start(nullptr, nullptr);
    if (gpioHandle < 0) {
        const char* error_msg = pigpio_error(gpioHandle);
        //std::cerr << "Error: " << error_msg << std::endl;
        return -1;
    }

    set_mode(gpioHandle, pins["AIN2"], PI_OUTPUT);
    set_mode(gpioHandle, pins["AIN1"], PI_OUTPUT);
    set_mode(gpioHandle, pins["STBY"], PI_OUTPUT);
    set_mode(gpioHandle, pins["BIN2"], PI_OUTPUT);
    set_mode(gpioHandle, pins["BIN1"], PI_OUTPUT);
    set_mode(gpioHandle, pins["PWMA"], PI_OUTPUT);
    set_mode(gpioHandle, pins["PWMB"], PI_OUTPUT);    
    
    set_mode(gpioHandle, pins["ENCA"], PI_INPUT);
    set_mode(gpioHandle, pins["ENCB"], PI_INPUT);

    callback_ex(gpioHandle, pins["ENCA"], EITHER_EDGE, callback_wrapper, &encA_);
    callback_ex(gpioHandle, pins["ENCB"], EITHER_EDGE, callback_wrapper, &encB_);

    auto time_ = std::chrono::system_clock::now();

    return 0;
}

int RPiComms::shutdown() {
    pigpio_stop(gpioHandle);
}

void RPiComms::callback_wrapper(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void* userdata) {
    //std::cout << "GPIO " << user_gpio << " changed to " << level << " at tick " << tick << std::endl;

    Encoder *enc = static_cast<Encoder*>(userdata);
    enc->value += 1 * enc->direction;
}

int RPiComms::read(double& l_wheel_vel, double& l_wheel_pos, double& r_wheel_vel, double& r_wheel_pos) {
    // Calculate time delta
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    double l_wheel_pos_prev = l_wheel_pos;
    l_wheel_pos = encA_.value * rads_per_tick;
    l_wheel_vel = (l_wheel_pos - l_wheel_pos_prev) / deltaSeconds;
    
    double r_wheel_pos_prev = r_wheel_pos;
    r_wheel_pos = encB_.value * rads_per_tick;
    r_wheel_vel = (r_wheel_pos - r_wheel_pos_prev) / deltaSeconds;

    return 0;
}

int RPiComms::write(double& l_wheel_cmd, double& r_wheel_cmd) {
    // set standby pin
    if (l_wheel_cmd == 0 && r_wheel_cmd == 0) {
        gpio_write(gpioHandle, pins["STBY"], PI_OFF);
    }
    else {
        gpio_write(gpioHandle, pins["STBY"], PI_ON);
    }
    
    // 63 = 25% duty cycle = 1.65 rev/s
    // 127 = 50% duty cycle = 3.5 rev/s
    // 191 = 75% duty cycle = 5.2 rev/s
    // 255 = 100% duty cycle = 6.1 rev/s

    // ratio of wheel velocity to pwm duty cycle
    double scalar = 36;

    // set l_wheel
    int l_wheel_speed = abs(l_wheel_cmd) * scalar;

    if (l_wheel_cmd > 0) {
        gpio_write(gpioHandle, pins["AIN1"], PI_ON);
        gpio_write(gpioHandle, pins["AIN2"], PI_OFF);
        set_PWM_dutycycle(gpioHandle, pins["PWMA"], l_wheel_speed);
        encA_.direction = 1;
    }
    else if (l_wheel_cmd < 0) {
        gpio_write(gpioHandle, pins["AIN1"], PI_OFF);
        gpio_write(gpioHandle, pins["AIN2"], PI_ON);
        set_PWM_dutycycle(gpioHandle, pins["PWMA"], l_wheel_speed);
        encA_.direction = -1;
    }
    else {
        gpio_write(gpioHandle, pins["AIN1"], PI_OFF);
        gpio_write(gpioHandle, pins["AIN2"], PI_OFF);
        set_PWM_dutycycle(gpioHandle, pins["PWMA"], 0);
        encA_.direction = 0;
    }

    // set r_wheel
    // assuming the same scalar as l_wheel for now
    int r_wheel_speed = abs(r_wheel_cmd) * scalar;

    if (r_wheel_cmd > 0) {
        gpio_write(gpioHandle, pins["BIN1"], PI_ON);
        gpio_write(gpioHandle, pins["BIN2"], PI_OFF);
        set_PWM_dutycycle(gpioHandle, pins["PWMB"], r_wheel_speed);
        encB_.direction = 1;
    }
    else if (r_wheel_cmd < 0) {
        gpio_write(gpioHandle, pins["BIN1"], PI_OFF);
        gpio_write(gpioHandle, pins["BIN2"], PI_ON);
        set_PWM_dutycycle(gpioHandle, pins["PWMB"], r_wheel_speed);
        encB_.direction = -1;
    }
    else {
        gpio_write(gpioHandle, pins["BIN1"], PI_OFF);
        gpio_write(gpioHandle, pins["BIN2"], PI_OFF);
        set_PWM_dutycycle(gpioHandle, pins["PWMB"], 0);
        encB_.direction = 0;
    }

    return 0;
}

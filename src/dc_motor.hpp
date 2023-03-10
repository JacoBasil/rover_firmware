
#ifndef DC_MOTOR
#define DC_MOTOR

#include "Arduino.h"
#include <ros.h>

#define MAX_PWM_VALUE 1023
const int16_t PWM_cutoff = 0;

//motor PINS
#define MOTOR_1_PWM 9
#define MOTOR_1_DIR 27
#define MOTOR_1_SLP 23

#define MOTOR_2_PWM 8
#define MOTOR_2_DIR 28
#define MOTOR_2_SLP 25

#define MOTOR_3_PWM 7
#define MOTOR_3_DIR 26
#define MOTOR_3_SLP 31

#define MOTOR_4_PWM 6
#define MOTOR_4_DIR 29
#define MOTOR_4_SLP 33

class DCMotor {
public:
    DCMotor(int8_t direction_pin, int8_t enable_pin, int8_t sleep_pin);
    void cw(uint16_t val);
    void ccw(uint16_t val);
    void stop();
private:
    int8_t _direction_pin;
    int8_t _enable_pin;
    int8_t _sleep_pin;
    uint16_t protectOutput(uint16_t val);
};

DCMotor::DCMotor(int8_t direction_pin, int8_t enable_pin, int8_t sleep_pin) {
    _direction_pin = direction_pin;
    _enable_pin = enable_pin;
    _sleep_pin = sleep_pin;
    //if (wiringPiSetupGpio() < 0) {
    //    ROS_ERROR("DCMotor wiringPi error: GPIO setup error");
    //    throw std::runtime_error("");
    //}
    //ROS_INFO("DCMotor wiringPi: GPIO setup");
    //pinMode(_direction_pin, OUTPUT);
    //pinMode(_enable_pin, OUTPUT);
    //stop();
    //ROS_INFO("DCMotor wiringPi: Motor setup");
}

void DCMotor::stop() {
    digitalWrite(_direction_pin, 0);
    digitalWrite(_sleep_pin, 0);
    analogWrite(_enable_pin, 0);
}

void DCMotor::cw(uint16_t val) {
    digitalWrite(_direction_pin, 1);
    digitalWrite(_sleep_pin, 1);
    analogWrite(_enable_pin, protectOutput(val));
}

void DCMotor::ccw(uint16_t val) {
    digitalWrite(_direction_pin, 0);
    digitalWrite(_sleep_pin, 1);
    analogWrite(_enable_pin, protectOutput(val));
}

uint16_t DCMotor::protectOutput(uint16_t val) {
    
    return val > MAX_PWM_VALUE ? MAX_PWM_VALUE : val;
}

#endif // DC_MOTOR_WIRING_PI_HPP_
#ifndef ENCODER_JETSON_HPP_
#define ENCODER_JETSON_HPP_

#include <ros/ros.h>
#include <JetsonGPIO.h>

#define ENCODER_1_PIN_A 24  // front left motor encoder
#define ENCODER_1_PIN_B 23  // CWW -
#define ENCODER_2_PIN_A 16  // front right motor encoder
#define ENCODER_2_PIN_B 15  // CW +
#define ENCODER_3_PIN_A 11  // rear left motor encoder
#define ENCODER_3_PIN_B 12  // CWW -
#define ENCODER_4_PIN_A 32  // rear right motor encoder
#define ENCODER_4_PIN_B 31  // CW +

#define PULSES_PER_REVOLUTION 3560

namespace EncoderPulseISR {

    volatile long encoderPosition1;
    volatile long encoderPosition2;
    volatile long encoderPosition3;
    volatile long encoderPosition4;
    volatile uint8_t encoderState1;
    volatile uint8_t encoderState2;
    volatile uint8_t encoderState3;
    volatile uint8_t encoderState4;

    void encoderISR(const int pinA, const int pinB, volatile long &encoderPosition, volatile uint8_t &encoderState) {
        uint8_t valA = GPIO::input(pinA);
        uint8_t valB = GPIO::input(pinB);
        uint8_t s = encoderState & 3;
        if (valA) s |= 4;
        if (valB) s |= 8; 
        encoderState = (s >> 2);
        if (s == 1 || s == 7 || s == 8 || s == 14)
            encoderPosition++;
        else if (s == 2 || s == 4 || s == 11 || s == 13)
            encoderPosition--;
        else if (s == 3 || s == 12)
            encoderPosition += 2;
        else if (s == 6 || s == 9)
            encoderPosition -= 2;
    }

    void encoderISR1(void) {
        encoderISR(ENCODER_1_PIN_A, ENCODER_1_PIN_B, encoderPosition1, encoderState1);
    }

    void encoderISR2(void) {
        encoderISR(ENCODER_2_PIN_A, ENCODER_2_PIN_B, encoderPosition2, encoderState2);
    }
    
    void encoderISR3(void) {
        encoderISR(ENCODER_3_PIN_A, ENCODER_3_PIN_B, encoderPosition3, encoderState4);
    }

    void encoderISR4(void) {
        encoderISR(ENCODER_4_PIN_A, ENCODER_4_PIN_B, encoderPosition4, encoderState4);
    }
}

class EncoderPulse {
public:
    EncoderPulse(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long* encoderPosition);
    double getAngle();
private:
    int _pinA;
    int _pinB;
    volatile long* _encoderPosition;
    double _initial_angle;
    double ticks2Angle(long position);
};

EncoderPulse::EncoderPulse(const int &pinA, const int &pinB, void (*isrFunction)(void), volatile long* encoderPosition) {
    _encoderPosition = encoderPosition;

    //if (wiringPiSetupSys() < 0) {
    //    ROS_ERROR("Encoder wiringPi error: GPIO setup error");
    //    throw std::runtime_error("");
    //}
    ROS_INFO("Jetson Nano::Encoder : GPIO setup");
    
    GPIO::setmode(GPIO::BOARD);
    _pinA = pinA;
    _pinB = pinB;
    GPIO::setup(_pinA, GPIO::IN);
    GPIO::setup(_pinB, GPIO::IN);

    //pinMode(_pinA, INPUT);
    //pinMode(_pinB, INPUT);
    //pullUpDnControl(_pinA, PUD_UP);
    //pullUpDnControl(_pinB, PUD_UP);

    GPIO::add_event_detect(_pinA, GPIO::BOTH, isrFunction);
    GPIO::add_event_detect(_pinB, GPIO::BOTH, isrFunction);

    /*
    if (wiringPiISR(_pinA, INT_EDGE_BOTH, isrFunction) < 0) {
        ROS_ERROR("Encoder wiringPi error: ISR pinA error");
        throw std::runtime_error("");
    }

    if (wiringPiISR(_pinB, INT_EDGE_BOTH, isrFunction) < 0) {
        ROS_ERROR("Encoder wiringPi error: ISR pinB error");
        throw std::runtime_error("");
    }
    */
    _initial_angle = ticks2Angle(*_encoderPosition);
    ROS_INFO("Jetson Nano::Encoder : ISR setup");
    
}

double EncoderPulse::getAngle() {
    double current_angle = ticks2Angle(*_encoderPosition);
    return current_angle - _initial_angle;
}

double EncoderPulse::ticks2Angle(long position) {
	return position * ((double)2 * M_PI / PULSES_PER_REVOLUTION / 2);
}

#endif 
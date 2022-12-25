#ifndef ENCODER_PULSE_HPP_
#define ENCODER_PULSE_HPP_
#define USE_USBCON
#include <Arduino.h>
#include <ros.h>

constexpr uint16_t PULSES_PER_REVOLUTION = 3440;
ros::NodeHandle node;

//encoder PINS
constexpr uint8_t ENCODER_1_PIN_A = 53;
constexpr uint8_t ENCODER_1_PIN_B = 52;
constexpr uint8_t ENCODER_2_PIN_A = 51;
constexpr uint8_t ENCODER_2_PIN_B = 50;
constexpr uint8_t ENCODER_3_PIN_A = 11;
constexpr uint8_t ENCODER_3_PIN_B = 12;
constexpr uint8_t ENCODER_4_PIN_A = 47;
constexpr uint8_t ENCODER_4_PIN_B = 46;

namespace EncoderPulseISR {

    volatile long  encoder_position_1;
	volatile long  encoder_position_2;
    volatile long  encoder_position_3;
    volatile long  encoder_position_4;
	volatile uint8_t encoder_state_1;
	volatile uint8_t encoder_state_2;
    volatile uint8_t encoder_state_3;
	volatile uint8_t encoder_state_4;

    void encoderISR(const int pin_A, const int pin_B, volatile long &encoder_position, volatile uint8_t &encoder_state) {
		uint8_t val_A = digitalRead(pin_A);
		uint8_t val_B = digitalRead(pin_B);
		uint8_t s = encoder_state & 3;
		if (val_A) s |= 4;
		if (val_B) s |= 8; 
		encoder_state = (s >> 2);
		if (s == 1 || s == 7 || s == 8 || s == 14)
			encoder_position++;
		else if (s == 2 || s == 4 || s == 11 || s == 13)
			encoder_position--;
		else if (s == 3 || s == 12)
			encoder_position += 2;
		else if (s == 6 || s == 9)
			encoder_position -= 2;
	}

	void encoderISR1(void) {
		encoderISR(ENCODER_1_PIN_A, ENCODER_1_PIN_B,  encoder_position_1, encoder_state_1);
	}

	void encoderISR2(void) {
		encoderISR(ENCODER_2_PIN_A, ENCODER_2_PIN_B,  encoder_position_2, encoder_state_2);
	}

    void encoderISR3(void) {
		encoderISR(ENCODER_3_PIN_A, ENCODER_3_PIN_B,  encoder_position_3, encoder_state_3);
	}

	void encoderISR4(void) {
		encoderISR(ENCODER_4_PIN_A, ENCODER_4_PIN_B,  encoder_position_4, encoder_state_4);
	}
}

class EncoderPulse {
public:
	EncoderPulse(const int &pin_A, const int &pin_B, void (*isrFunction)(void), volatile long* encoder_position);
	double getAngle();
private:
	int _pin_A;
	int _pin_B;
	volatile long* _encoder_position;
	double _initial_angle;
	double ticks2Angle(long position);
};

EncoderPulse::EncoderPulse(const int &pin_A, const int &pin_B, void (*isrFunction)(void), volatile long* encoder_position) {
	_encoder_position = encoder_position;
	_pin_A = pin_A;
	_pin_B = pin_B;
	_initial_angle = ticks2Angle(*_encoder_position);

}

double EncoderPulse::getAngle() {
	double current_angle = ticks2Angle(*_encoder_position);
	return current_angle - _initial_angle;
}

double EncoderPulse::ticks2Angle(long position) {
	return position * (double)2 * PI / (double)PULSES_PER_REVOLUTION;
}

#endif // ENCODER_PULSE


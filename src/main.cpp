#define USE_USBCON 
//#define _SAM3XA_
#include "encoder_pulse.hpp"
#include "dc_motor.hpp"
#include <std_msgs/Float64.h>

// 1ms interval for measurements
const long interval = 20;
long previousMillis = 0;
long currentMillis = 0;


ros::Time currentTime;
ros::Time previousTime;

std_msgs::Float64 _front_left_wheel_angle_msg;
std_msgs::Float64 _front_right_wheel_angle_msg;
std_msgs::Float64 _rear_left_wheel_angle_msg;
std_msgs::Float64 _rear_right_wheel_angle_msg;

std_msgs::Float64 _front_left_wheel_velocity_msg;
std_msgs::Float64 _front_right_wheel_velocity_msg;
std_msgs::Float64 _rear_left_wheel_velocity_msg;
std_msgs::Float64 _rear_right_wheel_velocity_msg;

ros::Publisher _front_left_wheel_angle_pub("/rover/front_left_wheel/angle", &_front_left_wheel_angle_msg);
ros::Publisher _front_right_wheel_angle_pub("/rover/front_right_wheel/angle", &_front_right_wheel_angle_msg);
ros::Publisher _rear_left_wheel_angle_pub("/rover/rear_left_wheel/angle", &_rear_left_wheel_angle_msg);
ros::Publisher _rear_right_wheel_angle_pub("/rover/rear_right_wheel/angle", &_rear_right_wheel_angle_msg);

ros::Publisher _front_left_wheel_velocity_pub("/rover/front_left_wheel/current_velocity", &_front_left_wheel_velocity_msg);	
ros::Publisher _front_right_wheel_velocity_pub("/rover/front_right_wheel/current_velocity", &_front_right_wheel_velocity_msg);	
ros::Publisher _rear_left_wheel_velocity_pub("/rover/rear_left_wheel/current_velocity", &_rear_left_wheel_velocity_msg);	
ros::Publisher _rear_right_wheel_velocity_pub("/rover/rear_right_wheel/current_velocity", &_rear_right_wheel_velocity_msg);	

EncoderPulse _encoder_front_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderPulseISR::encoderISR1, &EncoderPulseISR::encoder_position_1);
EncoderPulse _encoder_front_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderPulseISR::encoderISR2, &EncoderPulseISR::encoder_position_2);
EncoderPulse _encoder_rear_left(ENCODER_3_PIN_A, ENCODER_3_PIN_B, &EncoderPulseISR::encoderISR3, &EncoderPulseISR::encoder_position_3);
EncoderPulse _encoder_rear_right(ENCODER_4_PIN_A, ENCODER_4_PIN_B, &EncoderPulseISR::encoderISR4, &EncoderPulseISR::encoder_position_4);

DCMotor front_left_dc_motor(MOTOR_1_DIR, MOTOR_1_PWM, MOTOR_1_SLP);
DCMotor front_right_dc_motor(MOTOR_2_DIR, MOTOR_2_PWM, MOTOR_2_SLP);
DCMotor rear_left_dc_motor(MOTOR_3_DIR, MOTOR_3_PWM, MOTOR_3_SLP);
DCMotor rear_right_dc_motor(MOTOR_4_DIR, MOTOR_4_PWM, MOTOR_4_SLP);

double _front_left_wheel_angle;
double _front_right_wheel_angle;
double _rear_left_wheel_angle;
double _rear_right_wheel_angle;
	
double _front_left_wheel_velocity;
double _front_right_wheel_velocity;
double _rear_left_wheel_velocity;
double _rear_right_wheel_velocity;
	
double _front_left_wheel_position;
double _front_right_wheel_position;
double _rear_left_wheel_position;
double _rear_right_wheel_position;

void front_left_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > PWM_cutoff) {
		front_left_dc_motor.cw(abs(pwm));
	} else if (pwm < -PWM_cutoff) {
		front_left_dc_motor.ccw(abs(pwm));
	} else if (abs(pwm) <= PWM_cutoff) {
		front_left_dc_motor.stop();
	}
}

void front_right_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > PWM_cutoff) {
		front_right_dc_motor.ccw(abs(pwm));
	} else if (pwm < -PWM_cutoff) {
		front_right_dc_motor.cw(abs(pwm));
	} else if (abs(pwm) <= PWM_cutoff) {
		front_right_dc_motor.stop();
	}
}

void rear_left_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > PWM_cutoff) {
		rear_left_dc_motor.cw(abs(pwm));
	} else if (pwm < -PWM_cutoff) {
		rear_left_dc_motor.ccw(abs(pwm));
	} else if (abs(pwm) <= PWM_cutoff) {
		rear_left_dc_motor.stop();
	}
}

void rear_right_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > PWM_cutoff) {
		rear_right_dc_motor.ccw(abs(pwm));
	} else if (pwm < -PWM_cutoff) {
		rear_right_dc_motor.cw(abs(pwm));
	} else if (abs(pwm) <= PWM_cutoff) {
		rear_right_dc_motor.stop();
	}
}


ros::Subscriber<std_msgs::Float64> _front_left_wheel_target_vel_sub("/rover/front_left_wheel/pwm", &front_left_motor_callback);
ros::Subscriber<std_msgs::Float64> _front_right_wheel_target_vel_sub("/rover/front_right_wheel/pwm", &front_right_motor_callback);
ros::Subscriber<std_msgs::Float64> _rear_left_wheel_target_vel_sub("/rover/rear_left_wheel/pwm", &rear_left_motor_callback);
ros::Subscriber<std_msgs::Float64> _rear_right_wheel_target_vel_sub("/rover/rear_right_wheel/pwm", &rear_right_motor_callback);

void setup() {

	// ROS Setup
	node.getHardware()->setBaud(1000000);
	node.initNode();

	node.advertise(_front_left_wheel_angle_pub);
	node.advertise(_front_right_wheel_angle_pub); 
	node.advertise(_rear_left_wheel_angle_pub); 
	node.advertise(_rear_right_wheel_angle_pub); 
  	node.advertise(_front_left_wheel_velocity_pub);
  	node.advertise(_front_right_wheel_velocity_pub);
  	node.advertise(_rear_left_wheel_velocity_pub);
  	node.advertise(_rear_right_wheel_velocity_pub);
	node.subscribe(_front_left_wheel_target_vel_sub);
	node.subscribe(_front_right_wheel_target_vel_sub);
 	node.subscribe(_rear_left_wheel_target_vel_sub);
	node.subscribe(_rear_right_wheel_target_vel_sub);
	
	while (!node.connected())
  	{
    	node.spinOnce();
 	}

	node.loginfo("Connected to Arduino Due.");
	node.loginfo("Encoders::GPIO Setup");
	

	// Encoders PINS Setup
 	pinMode(ENCODER_1_PIN_A, INPUT_PULLUP);
	pinMode(ENCODER_1_PIN_B, INPUT_PULLUP);
	pinMode(ENCODER_2_PIN_A, INPUT_PULLUP);
	pinMode(ENCODER_2_PIN_B, INPUT_PULLUP);
	pinMode(ENCODER_3_PIN_A, INPUT_PULLUP);
	pinMode(ENCODER_3_PIN_B, INPUT_PULLUP);
	pinMode(ENCODER_4_PIN_A, INPUT_PULLUP);
	pinMode(ENCODER_4_PIN_B, INPUT_PULLUP);

	node.loginfo("DCMotors::GPIO Setup");
	analogWriteResolution(10);
	pinMode(MOTOR_1_PWM, OUTPUT);
	pinMode(MOTOR_1_DIR, OUTPUT);
	pinMode(MOTOR_1_SLP, OUTPUT);
	pinMode(MOTOR_2_PWM, OUTPUT);
	pinMode(MOTOR_2_DIR, OUTPUT);
	pinMode(MOTOR_2_SLP, OUTPUT);
	pinMode(MOTOR_3_PWM, OUTPUT);
	pinMode(MOTOR_3_DIR, OUTPUT);
	pinMode(MOTOR_3_SLP, OUTPUT);
	pinMode(MOTOR_4_PWM, OUTPUT);
	pinMode(MOTOR_4_DIR, OUTPUT);
	pinMode(MOTOR_4_SLP, OUTPUT);
	
	node.loginfo("Encoders::ISR Initialization");
  	attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_A), EncoderPulseISR::encoderISR1, CHANGE);
  	attachInterrupt(digitalPinToInterrupt(ENCODER_1_PIN_B), EncoderPulseISR::encoderISR1, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_A), EncoderPulseISR::encoderISR2, CHANGE);
  	attachInterrupt(digitalPinToInterrupt(ENCODER_2_PIN_B), EncoderPulseISR::encoderISR2, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_3_PIN_A), EncoderPulseISR::encoderISR3, CHANGE);
  	attachInterrupt(digitalPinToInterrupt(ENCODER_3_PIN_B), EncoderPulseISR::encoderISR3, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ENCODER_4_PIN_A), EncoderPulseISR::encoderISR4, CHANGE);
  	attachInterrupt(digitalPinToInterrupt(ENCODER_4_PIN_B), EncoderPulseISR::encoderISR4, CHANGE);
	//previousMillis = micros();
	//previousMillisForSpin = micros();
}

void loop() {

	currentMillis = millis();
	//currentTime = node.now()//ros::Duration elapsed_time = currentTime - previousTime;

	double elapsed_time = ((double)currentMillis - (double)previousMillis)/ 1000.0;
	_front_left_wheel_angle = 1 * _encoder_front_left.getAngle();
	_front_right_wheel_angle = -1 * _encoder_front_right.getAngle();
	_rear_left_wheel_angle = 1 * _encoder_rear_left.getAngle();
	_rear_right_wheel_angle = -1 * _encoder_rear_right.getAngle();

	_front_left_wheel_angle_msg.data = _front_left_wheel_angle;
	_front_right_wheel_angle_msg.data = _front_right_wheel_angle;
	_rear_left_wheel_angle_msg.data = _rear_left_wheel_angle;
	_rear_right_wheel_angle_msg.data = _rear_right_wheel_angle;

	double delta_front_left_wheel = _front_left_wheel_angle - _front_left_wheel_position;
	double delta_front_right_wheel = _front_right_wheel_angle - _front_right_wheel_position;
	double delta_rear_left_wheel = _rear_left_wheel_angle - _rear_left_wheel_position;
	double delta_rear_right_wheel = _rear_right_wheel_angle - _rear_right_wheel_position;

	_front_left_wheel_velocity = delta_front_left_wheel / elapsed_time;//elapsed_time.toSec();
	_front_right_wheel_velocity = delta_front_right_wheel / elapsed_time;//elapsed_time.toSec();
	_rear_left_wheel_velocity = delta_rear_left_wheel / elapsed_time;//elapsed_time.toSec();
	_rear_right_wheel_velocity = delta_rear_right_wheel / elapsed_time;//elapsed_time.toSec();

	// Convert to RPM
	_front_left_wheel_velocity_msg.data = _front_left_wheel_velocity; 
	_front_right_wheel_velocity_msg.data = _front_right_wheel_velocity;
	_rear_left_wheel_velocity_msg.data = _rear_left_wheel_velocity;
	_rear_right_wheel_velocity_msg.data= _rear_right_wheel_velocity;

	if (currentMillis - previousMillis > interval) {
    	previousMillis = currentMillis;
		//previousTime = currentTime;

		_front_left_wheel_position += delta_front_left_wheel;
		_front_right_wheel_position += delta_front_right_wheel;
		_rear_left_wheel_position += delta_rear_left_wheel;
		_rear_right_wheel_position += delta_rear_right_wheel;

    	_front_left_wheel_angle_pub.publish(&_front_left_wheel_angle_msg );
		_front_right_wheel_angle_pub.publish(&_front_right_wheel_angle_msg);
		_rear_left_wheel_angle_pub.publish(&_rear_left_wheel_angle_msg);
		_rear_right_wheel_angle_pub.publish(&_rear_right_wheel_angle_msg);

		_front_left_wheel_velocity_pub.publish(&_front_left_wheel_velocity_msg);
		_front_right_wheel_velocity_pub.publish(&_front_right_wheel_velocity_msg);
		_rear_left_wheel_velocity_pub.publish(&_rear_left_wheel_velocity_msg);
		_rear_right_wheel_velocity_pub.publish(&_rear_right_wheel_velocity_msg);
		node.spinOnce();
	}
}

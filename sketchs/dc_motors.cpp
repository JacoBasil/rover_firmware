#define USE_USBCON 
#include "dc_motor.hpp"
#include <std_msgs/Float64.h>

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

ros::NodeHandle node;

DCMotor front_left_dc_motor(MOTOR_1_DIR, MOTOR_1_PWM, MOTOR_1_SLP);
DCMotor front_right_dc_motor(MOTOR_2_DIR, MOTOR_2_PWM, MOTOR_2_SLP);
DCMotor rear_left_dc_motor(MOTOR_3_DIR, MOTOR_3_PWM, MOTOR_3_SLP);
DCMotor rear_right_dc_motor(MOTOR_4_DIR, MOTOR_4_PWM, MOTOR_4_SLP);

// 10ms interval for measurements
const int interval = 10;
long previousMillis = 0;
long currentMillis = 0;


ros::Time currentTime;
ros::Time previousTime;

void front_left_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = 800;
	if (pwm > 0) {
		front_left_dc_motor.cw(abs(pwm));
	} else if (pwm < 0) {
		front_left_dc_motor.ccw(abs(pwm));
	} else if (pwm == 0) {
		front_left_dc_motor.stop();
	}
}

void front_right_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > 0) {
		front_right_dc_motor.ccw(abs(pwm));
	} else if (pwm < 0) {
		front_right_dc_motor.cw(abs(pwm));
	} else if (pwm == 0) {
		front_right_dc_motor.stop();
	}
}

void rear_left_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > 0) {
		rear_left_dc_motor.ccw(abs(pwm));
	} else if (pwm < 0) {
		rear_left_dc_motor.cw(abs(pwm));
	} else if (pwm == 0) {
		rear_left_dc_motor.stop();
	}
}

void rear_right_motor_callback(const std_msgs::Float64 &msg) {
	int16_t pwm = msg.data * 100;
	if (pwm > 0) {
		rear_right_dc_motor.ccw(abs(pwm));
	} else if (pwm < 0) {
		rear_right_dc_motor.cw(abs(pwm));
	} else if (pwm == 0) {
		rear_right_dc_motor.stop();
	}
}

ros::Subscriber<std_msgs::Float64> _front_left_wheel_target_vel_sub("/rover/front_left_wheel/pwm", &front_left_motor_callback);
ros::Subscriber<std_msgs::Float64> _front_right_wheel_target_vel_sub("/rover/front_right_wheel/pwm", &front_right_motor_callback);
ros::Subscriber<std_msgs::Float64> _rear_left_wheel_target_vel_sub("/rover/rear_left_wheel/pwm", &rear_left_motor_callback);
ros::Subscriber<std_msgs::Float64> _rear_right_wheel_target_vel_sub("/rover/rear_right_wheel/pwm", &rear_right_motor_callback);

void setup(){
	
	// ROS Setup
  	node.getHardware()->setBaud(115200);
  	node.initNode();
 	node.subscribe(_front_left_wheel_target_vel_sub);
	node.subscribe(_front_right_wheel_target_vel_sub);
 	node.subscribe(_rear_left_wheel_target_vel_sub);
	node.subscribe(_rear_right_wheel_target_vel_sub);
	node.loginfo("Motors GPIO Setup");
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
	previousTime = node.now();
}

void loop(){
	currentTime = node.now();
	ros::Duration elapsed_time = currentTime - previousTime;
	// Record the time
  	//currentMillis = millis();
	if (elapsed_time.toSec() > interval / 1000) {
    	//previousMillis = currentMillis;
		previousTime = currentTime;
    	node.spinOnce();
  	}
	//front_left_dc_motor.ccw(abs(pwm_1));
}
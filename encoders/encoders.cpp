#include "encoder_jetson.hpp"
#include <chrono>
#include <std_msgs/Float64.h>

typedef boost::chrono::steady_clock time_source;

class Encoders {
public:
	Encoders(double update_rate);

private:
	ros::NodeHandle node;

	ros::Publisher _front_left_wheel_angle_pub;
	ros::Publisher _front_right_wheel_angle_pub;
	ros::Publisher _front_left_wheel_velocity_pub;
	ros::Publisher _front_right_wheel_velocity_pub;
	ros::Publisher _rear_left_wheel_angle_pub;
	ros::Publisher _rear_right_wheel_angle_pub;
	ros::Publisher _rear_left_wheel_velocity_pub;
	ros::Publisher _rear_right_wheel_velocity_pub;

	ros::Timer encoders_timer;

	std_msgs::Float64 front_left_wheel_angle_msg;
	std_msgs::Float64 front_right_wheel_angle_msg;
	std_msgs::Float64 front_left_wheel_velocity_msg;
	std_msgs::Float64 front_right_wheel_velocity_msg;
	std_msgs::Float64 rear_left_wheel_angle_msg;
	std_msgs::Float64 rear_right_wheel_angle_msg;
	std_msgs::Float64 rear_left_wheel_velocity_msg;
	std_msgs::Float64 rear_right_wheel_velocity_msg;

	EncoderPulse encoder_front_left;
	EncoderPulse encoder_front_right;
	EncoderPulse encoder_rear_left;
	EncoderPulse encoder_rear_right;

	double front_left_wheel_angle;
	double front_right_wheel_angle;
	double rear_left_wheel_angle;
	double rear_right_wheel_angle;
	
	double front_left_wheel_velocity;
	double front_right_wheel_velocity;
	double rear_left_wheel_velocity;
	double rear_right_wheel_velocity;
	
	double front_left_wheel_position;
	double front_right_wheel_position;
	double rear_left_wheel_position;
	double rear_right_wheel_position;

	time_source::time_point last_time;

	void encodersCallback(const ros::TimerEvent& event);
};

Encoders::Encoders(double update_rate)
	: encoder_front_left(ENCODER_1_PIN_A, ENCODER_1_PIN_B, &EncoderPulseISR::encoderISR1, &EncoderPulseISR::encoderPosition1)
	, encoder_front_right(ENCODER_2_PIN_A, ENCODER_2_PIN_B, &EncoderPulseISR::encoderISR2, &EncoderPulseISR::encoderPosition2)
	, encoder_rear_left(ENCODER_3_PIN_A, ENCODER_3_PIN_B, &EncoderPulseISR::encoderISR3, &EncoderPulseISR::encoderPosition3)
	, encoder_rear_right(ENCODER_4_PIN_A, ENCODER_4_PIN_B, &EncoderPulseISR::encoderISR4, &EncoderPulseISR::encoderPosition4) {
	_front_left_wheel_angle_pub = node.advertise<std_msgs::Float64>("/rover/front_left_wheel/angle", 1);
	_front_right_wheel_angle_pub = node.advertise<std_msgs::Float64>("/rover/front_right_wheel/angle", 1);
	_front_left_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/rover/front_left_wheel/current_velocity", 1);
	_front_right_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/rover/front_right_wheel/current_velocity", 1);
	_rear_left_wheel_angle_pub = node.advertise<std_msgs::Float64>("/rover/rear_left_wheel/angle", 1);
	_rear_right_wheel_angle_pub = node.advertise<std_msgs::Float64>("/rover/rear_right_wheel/angle", 1);
	_rear_left_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/rover/rear_left_wheel/current_velocity", 1);
	_rear_right_wheel_velocity_pub = node.advertise<std_msgs::Float64>("/rover/rear_right_wheel/current_velocity", 1);

	encoders_timer = node.createTimer(ros::Duration(update_rate), &Encoders::encodersCallback, this);
}

void Encoders::encodersCallback(const ros::TimerEvent& event) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	front_left_wheel_angle = -1 * encoder_front_left.getAngle();
	front_right_wheel_angle = 1 * encoder_front_right.getAngle();
	rear_left_wheel_angle = -1 * encoder_rear_left.getAngle();
	rear_right_wheel_angle = 1 * encoder_rear_right.getAngle();

	front_left_wheel_angle_msg.data = front_left_wheel_angle;
	front_right_wheel_angle_msg.data = front_right_wheel_angle;
	rear_left_wheel_angle_msg.data = rear_left_wheel_angle;
	rear_right_wheel_angle_msg.data = rear_right_wheel_angle;

	_front_left_wheel_angle_pub.publish(front_left_wheel_angle_msg);
	_front_right_wheel_angle_pub.publish(front_right_wheel_angle_msg);
	_rear_left_wheel_angle_pub.publish(rear_left_wheel_angle_msg);
	_rear_right_wheel_angle_pub.publish(rear_right_wheel_angle_msg);

	double front_delta_left_wheel = front_left_wheel_angle - front_left_wheel_position;
	double front_delta_right_wheel = front_right_wheel_angle - front_right_wheel_position;
	double rear_delta_left_wheel = rear_left_wheel_angle - rear_left_wheel_position;
	double rear_delta_right_wheel = rear_right_wheel_angle - rear_right_wheel_position;

	front_left_wheel_position += front_delta_left_wheel;
	front_left_wheel_velocity = front_delta_left_wheel / elapsed.toSec();

	front_right_wheel_position += front_delta_right_wheel;
	front_right_wheel_velocity = front_delta_right_wheel / elapsed.toSec();

	rear_left_wheel_position += rear_delta_left_wheel;
	rear_left_wheel_velocity = rear_delta_left_wheel / elapsed.toSec();

	rear_right_wheel_position += rear_delta_right_wheel;
	rear_right_wheel_velocity = rear_delta_right_wheel / elapsed.toSec();

	front_left_wheel_velocity_msg.data = front_left_wheel_velocity;
	front_right_wheel_velocity_msg.data = front_right_wheel_velocity;
	rear_left_wheel_velocity_msg.data = rear_left_wheel_velocity;
	rear_right_wheel_velocity_msg.data = rear_right_wheel_velocity;

	_front_left_wheel_velocity_pub.publish(front_left_wheel_velocity_msg);
	_front_right_wheel_velocity_pub.publish(front_right_wheel_velocity_msg);
	_rear_left_wheel_velocity_pub.publish(rear_left_wheel_velocity_msg);
	_rear_right_wheel_velocity_pub.publish(rear_right_wheel_velocity_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "encoders");
	Encoders encoders(0.01);
	ros::spin();
	return 0;
}
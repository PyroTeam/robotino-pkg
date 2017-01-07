/*
 * JoystickTeleop.h
 *
 *  Created on: 16.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef JOYSTICKTELEOP_H_
#define JOYSTICKTELEOP_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <gripper_msg/Grip.h>

class JoystickTeleop
{
public:
	JoystickTeleop();
	~JoystickTeleop();

private:
	ros::NodeHandle nh_;

	ros::Publisher cmd_vel_pub_;
	ros::Subscriber joy_sub_;
	ros::ServiceClient m_gripper_client;

	geometry_msgs::Twist cmd_vel_msg_;
	gripper_msg::Grip m_gripper_srv;

	void readParams( ros::NodeHandle& n );
	void joyCallback( const sensor_msgs::JoyConstPtr& msg);

	// params
	int axis_linear_x_;
	int axis_linear_y_;
	int axis_angular_;
	int m_button_r1;
	int m_button_l1;
	double scale_linear_;
	double scale_angular_;

	bool m_last_r1;
	bool m_last_l1;
	gripper_msg::GripRequest m_last_req;

public:
	void spin();
};


#endif /* JOYSTICKTELEOP_H_ */

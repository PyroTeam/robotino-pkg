/*
 * JoystickTeleop.cpp
 *
 *  Created on: 16.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "JoystickTeleop.h"

JoystickTeleop::JoystickTeleop()
: nh_("~")
, m_last_r1(false)
, m_last_l1(false)
{
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	joy_sub_ = nh_.subscribe("/joy", 1, &JoystickTeleop::joyCallback, this);
	m_gripper_client = nh_.serviceClient<gripper_msg::Grip>("/gripper_srv");
	m_last_req.cmd = gripper_msg::GripRequest::NOTHING;

	readParams( nh_ );
}

JoystickTeleop::~JoystickTeleop()
{
	cmd_vel_pub_.shutdown();
	joy_sub_.shutdown();
}

void JoystickTeleop::readParams( ros::NodeHandle& n)
{
	n.param<int>("axis_linear_x", axis_linear_x_, 1);
	n.param<int>("axis_linear_y", axis_linear_y_, 0);
	n.param<int>("axis_angular", axis_angular_, 2);
	n.param<int>("button_r1", m_button_r1, 5);
	n.param<int>("button_l1", m_button_l1, 4);
	n.param<double>("scale_linear", scale_linear_, 0.1);
	n.param<double>("scale_angular", scale_angular_, 0.2);

}

void JoystickTeleop::joyCallback( const sensor_msgs::JoyConstPtr& msg)
{
	if( msg->axes.size() < 3)
	{
		ROS_ERROR( "Too few joystick axes: %lu (expected more than 3)", msg->axes.size() );
		return;
	}

	cmd_vel_msg_.linear.x = msg->axes[axis_linear_x_] * scale_linear_;
	cmd_vel_msg_.linear.y = msg->axes[axis_linear_y_] * scale_linear_;
	cmd_vel_msg_.angular.z = msg->axes[axis_angular_] * scale_angular_;


	// Click on Right-1 button - have priority on L1 click
	if (m_last_r1 != msg->buttons[m_button_r1] && msg->buttons[m_button_r1])
	{
		m_gripper_srv.request.cmd = gripper_msg::GripRequest::TAKE;
	}
	// Click on Left-1 button
	else if (m_last_l1 != msg->buttons[m_button_l1] && msg->buttons[m_button_l1])
	{
		m_gripper_srv.request.cmd = gripper_msg::GripRequest::LET;
	}
	else
	{
		m_gripper_srv.request.cmd = gripper_msg::GripRequest::NOTHING;
	}
	m_last_r1 = (bool)msg->buttons[m_button_r1];
	m_last_l1 = (bool)msg->buttons[m_button_l1];
}

void JoystickTeleop::spin()
{
	ros::Rate loop_rate(10);
	while( nh_.ok() )
	{
		cmd_vel_pub_.publish( cmd_vel_msg_ );
		ros::spinOnce();
		loop_rate.sleep();
		if (m_gripper_srv.request.cmd != m_last_req.cmd
			&& m_gripper_srv.request.cmd != gripper_msg::GripRequest::NOTHING)
		{
			m_gripper_client.call(m_gripper_srv);
		}
		m_last_req.cmd = m_gripper_srv.request.cmd;
	}
}

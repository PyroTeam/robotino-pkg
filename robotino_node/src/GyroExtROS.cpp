/**
 * \file 		GyroExtROS.cpp
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2017-05-31
 * \copyright   2017, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#include "GyroExtROS.h"

GyroExtROS::GyroExtROS()
{
    gyro_pub_ = nh_.advertise<robotino_msgs::GyroReadings>("gyro", 1, false);
}

GyroExtROS::~GyroExtROS()
{
	gyro_pub_.shutdown();
}

void GyroExtROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void GyroExtROS::gyroscopeExtEvent(float angle, float rate)
{
	// Build the Gyro message
	gyro_msg_.header.stamp = stamp_;
	gyro_msg_.header.frame_id = "base_link";

    gyro_msg_.angle = angle;
    gyro_msg_.rate = rate;

    gyro_pub_.publish(gyro_msg_);
}

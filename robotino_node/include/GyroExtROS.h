/**
 * \file 		GyroExtROS.h
 *
 * \brief
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date		2017-05-31
 * \copyright   2017, Association de Robotique de Polytech Lille All rights reserved
 * \version
 */

#ifndef GYROEXTROS_H_
#define GYROEXTROS_H_

#include "rec/robotino/api2/GyroscopeExt.h"

#include <ros/ros.h>
#include "robotino_msgs/GyroReadings.h"

class GyroExtROS: public rec::robotino::api2::GyroscopeExt
{
public:
	GyroExtROS();
	~GyroExtROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher gyro_pub_;

    robotino_msgs::GyroReadings gyro_msg_;

	ros::Time stamp_;

	void gyroscopeExtEvent(float angle, float rate);
};

#endif /* GYROEXTROS_H_ */

/*
 * PowerManagementROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "PowerManagementROS.h"

PowerManagementROS::PowerManagementROS()
{
	power_pub_ = nh_.advertise<robotino_msgs::PowerReadings>("power_readings", 1, true);
}

PowerManagementROS::~PowerManagementROS()
{
	power_pub_.shutdown();
}

void PowerManagementROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

//void PowerManagementROS::readingsEvent(float current, float voltage)
void PowerManagementROS::readingsEvent( float battery_voltage, float system_current, bool ext_power, int num_chargers, const char* batteryType, bool batteryLow, int batteryLowShutdownCounter )
{
	// Build the PowerReadings msg
	power_msg_.stamp = ros::Time::now();
	//power_msg_.current = current;
	//power_msg_.voltage = voltage;

	power_msg_.battery_voltage = battery_voltage;
        power_msg_.system_current = system_current;
	power_msg_.ext_power = ext_power;
        power_msg_.num_chargers = num_chargers;
        power_msg_.batteryLow = batteryLow;
        power_msg_.batteryLowShutdownCounter = batteryLowShutdownCounter;

	// Publish the msg
	power_pub_.publish( power_msg_ );
}

/*
 * CompactBHAROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "CompactBHAROS.h"

CompactBHAROS::CompactBHAROS()
{
	bha_pub_ = nh_.advertise<robotino_msgs::BHAReadings>("bha_readings", 1, true);
	bha_sub_ = nh_.subscribe("set_bha_pressures", 1, &CompactBHAROS::setBHAPressuresCallback, this);
	set_compressor_enabled_server_ = nh_.advertiseService("set_compressor_enabled",
			&CompactBHAROS::setCompressorsEnabledService, this);
}

CompactBHAROS::~CompactBHAROS()
{
	bha_pub_.shutdown();
	bha_sub_.shutdown();
}

void CompactBHAROS::setTimeStamp(ros::Time stamp)
{
	stamp_ = stamp;
}

void CompactBHAROS::pressuresChangedEvent( const float* pressures, unsigned int size )
{
	// Build the BHAReadings msg
	bha_msg_.pressures.resize( size, 0.0 );

	if( pressures != NULL )
	{
		memcpy( bha_msg_.pressures.data(), pressures, size * sizeof(float) );
	}
}

void CompactBHAROS::pressureSensorChangedEvent( bool pressureSensor )
{
	bha_msg_.pressureSensor = pressureSensor;
}

void CompactBHAROS::stringPotsChangedEvent( const float* readings, unsigned int size )
{
	// Build the BHAReadings msg
	bha_msg_.stringPots.resize( size, 0.0 );
	if( readings != NULL )
	{
		memcpy( bha_msg_.stringPots.data(), readings, size * sizeof(float) );
	}

	// Publish the msg
	bha_pub_.publish( bha_msg_ );
}

void CompactBHAROS::foilPotChangedEvent( float value )
{
	bha_msg_.foilPot = value;
}


void CompactBHAROS::setBHAPressuresCallback(const robotino_msgs::SetBHAPressuresConstPtr &msg)
{
	float pressures[8];

	if( msg->pressures.size() == 8 )
	{
		for(int i = 0; i < 8; ++i)
		{
			pressures[i] = msg->pressures[i];
		}

		setPressures( pressures );
	}
}




bool CompactBHAROS::setCompressorsEnabledService(
	robotino_msgs::SetCompressorsEnabled::Request &req,
	robotino_msgs::SetCompressorsEnabled::Response &res)
{
	setCompressorsEnabled( req.enable );
	return true;
}

/*
 * CompactBHAROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef COMPACTBHAROS_H_
#define COMPACTBHAROS_H_

#include "rec/robotino/api2/CompactBHA.h"

#include <ros/ros.h>
#include "robotino_msgs/BHAReadings.h"
#include "robotino_msgs/SetBHAPressures.h"
#include "robotino_msgs/SetCompressorsEnabled.h"

class CompactBHAROS : public rec::robotino::api2::CompactBHA
{
public:
	CompactBHAROS();
	~CompactBHAROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Subscriber bha_sub_;

	ros::Publisher bha_pub_;

    ros::ServiceServer set_compressor_enabled_server_;

    /* TODO(vcoelen) : add these functionnalities (cf CompactBHA.h in API2 source)
    setWaterDrainValve
    setGripperValve1
    setGripperValve2
    */

    robotino_msgs::BHAReadings bha_msg_;

	ros::Time stamp_;

	void pressuresChangedEvent( const float* pressures, unsigned int size );
    void pressureSensorChangedEvent( bool pressureSensor );
    void stringPotsChangedEvent( const float* readings, unsigned int size );
    void foilPotChangedEvent( float value );

	void setBHAPressuresCallback( const robotino_msgs::SetBHAPressuresConstPtr &msg);

    bool setCompressorsEnabledService(
        robotino_msgs::SetCompressorsEnabled::Request &req,
        robotino_msgs::SetCompressorsEnabled::Response &res);
};

#endif /* COMPACTBHAROS_H_ */

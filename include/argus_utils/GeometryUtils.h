#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <tf/LinearMath/Transform.h> // From tf

#include "argus_utils/PoseSE3.h"

#include <Eigen/Geometry>
#include <iostream>

namespace argus_utils
{
	
// TODO Templatize scalar type
struct EulerAngles
{
	double yaw;
	double pitch;
	double roll;
	
	EulerAngles( double y = 0, double p = 0, double r = 0 );
};

// Convert between Eigen quaternions and yaw-pitch-roll Euler angles
EulerAngles QuaternionToEuler( const Eigen::Quaterniond& quat );
Eigen::Quaterniond EulerToQuaternion( const EulerAngles& eul );

// Convert between PoseSE3 and tf Transform objects
tf::Transform PoseToTf( const PoseSE3& pose );
PoseSE3 TfToPose( const tf::Transform& tf );

// Convert between geometry_msgs::Pose message and PoseSE3 objects
geometry_msgs::Pose PoseToMsg( const PoseSE3& pose );
PoseSE3 MsgToPose( const geometry_msgs::Pose& msg );

// Convert between geometry_msgs::Twist message and PoseSE3::TangentVector
geometry_msgs::Twist TangentToMsg( const PoseSE3::TangentVector& tan );
PoseSE3::TangentVector MsgToTangent( const geometry_msgs::Twist& twist );

// Print an Euler angle in "Y: yaw P: pitch R: roll" format
std::ostream& operator<<( std::ostream& os, const EulerAngles& eul );
	
} // end namespace argus_utils

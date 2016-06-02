#pragma once

#include <Eigen/Geometry>

namespace argus
{

// Geometry types

// Standard quaternion
typedef Eigen::Quaterniond QuaternionType;

// 3D translation
typedef Eigen::Translation<double, 3> Translation3Type;

// 2D translation
typedef Eigen::Translation<double, 2> Translation2Type;

// Non-ambiguous Euler angles tuple
// Convention is ZYX body rotations
struct EulerAngles
{
	double yaw;
	double pitch;
	double roll;
	
	EulerAngles( double y = 0, double p = 0, double r = 0 ) 
	: yaw( y ), pitch ( p ), roll ( r ) {}
};

}
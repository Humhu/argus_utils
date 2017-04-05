#pragma once

#include <argus_utils/geometry/PoseSE3.h>
#include "geometry_msgs/TransformStamped.h"

#include <exception>

namespace argus
{

// Exception denoting an exception lookup error
class ExtrinsicsException : public std::exception
{
public:

	ExtrinsicsException( const std::string& msg );

	virtual const char* what() const throw();

private:

	std::string _msg;
};

struct RelativePose
{
	ros::Time time;
	std::string parentID;
	std::string childID;
	PoseSE3 pose;

	RelativePose();
	RelativePose( const std::string& parent, const std::string& child, 
	              const PoseSE3& pose, const ros::Time& time = ros::Time() );
	RelativePose( const geometry_msgs::TransformStamped& msg );
	
	geometry_msgs::TransformStamped ToTransformMsg() const;
};

}
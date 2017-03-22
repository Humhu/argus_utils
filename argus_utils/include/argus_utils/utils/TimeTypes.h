#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>

namespace argus
{

// Time types
// Compatible with ros::Time, but keeps internal libraries ROS-free
typedef boost::posix_time::ptime Time;
typedef boost::posix_time::time_duration TimeDuration;

inline double to_seconds( const TimeDuration& td )
{
	return td.total_nanoseconds()*1E-9;
}

}
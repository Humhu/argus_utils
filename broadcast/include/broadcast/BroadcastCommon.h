#pragma once

#include "broadcast/FloatVectorStamped.h"
#include "broadcast/QueryFeatures.h"
#include "argus_utils/utils/LinalgTypes.h"
#include <string>

namespace argus
{

enum BroadcastMode
{
	PUSH_TOPIC,
	PULL_TOPIC
};

std::string BroadcastModeToString( BroadcastMode mode );
BroadcastMode StringToBroadcastMode( const std::string& str );

enum QueryMode
{
	CLOSEST_BEFORE = broadcast::QueryFeatures::Request::CLOSEST_BEFORE,
	CLOSEST_AFTER = broadcast::QueryFeatures::Request::CLOSEST_AFTER,
	CLOSEST_EITHER = broadcast::QueryFeatures::Request::CLOSEST_EITHER
};

std::string QueryModeToString( QueryMode mode );
QueryMode StringToQueryMode( const std::string& str );

struct StampedFeatures
{
	ros::Time time;
	std::string name;
	VectorType features;

	StampedFeatures();
	StampedFeatures( const ros::Time& t, const std::string& n, const VectorType& v );
	StampedFeatures( const broadcast::FloatVectorStamped& msg );
	broadcast::FloatVectorStamped ToMsg() const;
};

}
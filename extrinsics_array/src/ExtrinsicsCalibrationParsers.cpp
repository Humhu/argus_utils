#include "extrinsics_array/ExtrinsicsCalibrationParsers.h"

#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <fstream>

namespace argus
{

bool ParseExtrinsicsCalibration( const YAML::Node& yaml, std::vector<RelativePose>& poses )
{
	poses.clear();

	YAML::Node::const_iterator iter;
	for( iter = yaml.begin(); iter != yaml.end(); ++iter )
	{
		RelativePose pose;
		YAML::Node info = iter->second;

		if( !GetParam( info, "frame_id", pose.childID ) )
		{
			pose.childID = iter->first.as<std::string>();
		}
		try
		{
			GetParamRequired( info, "parent_id", pose.parentID );
			GetParamRequired( info, "pose", pose.pose );
		}
		catch( std::invalid_argument )
		{
			poses.clear();
			return false;
		}
		poses.push_back( pose );
	}
	return true;
}

void PopulateExtrinsicsCalibration( const RelativePose& pose, YAML::Node& yaml )
{
	yaml["parent_id"] = pose.parentID;
	yaml["pose"] = SetPoseYaml( pose.pose );
}

void PopulateExtrinsicsCalibration( const std::vector<RelativePose>& poses, YAML::Node& yaml )
{
	BOOST_FOREACH( const RelativePose &p, poses )
	{
		std::string parentPrefix = p.parentID + "/";
		size_t pos = p.childID.find( parentPrefix );
		std::string childSubID = p.childID;
		if( pos == 0 )
		{
			childSubID.replace( 0, parentPrefix.size(), "" );
		}

		YAML::Node subnode;
		subnode["parent_id"] = p.parentID;
		subnode["pose"] = SetPoseYaml( p.pose );
		subnode["frame_id"] = childSubID;
		yaml[childSubID] = subnode;
	}
}

bool ReadExtrinsicsCalibration( const std::string& path, std::vector<RelativePose>& poses )
{
	YAML::Node yaml;
	try 
	{
		 yaml = YAML::LoadFile( path );
	}
	catch( YAML::BadFile e )
	{
		return false;
	}
	
	return ParseExtrinsicsCalibration( yaml, poses );
}

bool WriteExtrinsicsCalibration( const std::string& path, const std::vector<RelativePose>& poses )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	PopulateExtrinsicsCalibration( poses, yaml );
	output << yaml;
	return true;
}

}
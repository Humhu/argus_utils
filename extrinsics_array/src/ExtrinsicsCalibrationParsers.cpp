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
		pose.childID = iter->first.as<std::string>();
		YAML::Node info = iter->second;
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
		YAML::Node subnode;
		subnode["parent_id"] = p.parentID;
		subnode["pose"] = SetPoseYaml( p.pose );
		yaml[p.childID] = subnode;
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
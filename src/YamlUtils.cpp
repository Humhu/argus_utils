#include "argus_utils/YamlUtils.h"
#include "argus_utils/MatrixUtils.h"

namespace argus_utils
{

// Needed because int strings like '0' do not convert to doubles automatically
double ParseDouble( XmlRpc::XmlRpcValue& xml )
{
	if( xml.getType() == XmlRpc::XmlRpcValue::TypeInt )
	{
		return (double) static_cast<int>( xml );
	}
	return static_cast<double>( xml );
}

bool CheckXmlType( const XmlRpc::XmlRpcValue& xml, XmlRpc::XmlRpcValue::Type type )
{
	if( xml.getType() == XmlRpc::XmlRpcValue::TypeArray )
	{
		for( int i = 0; i < xml.size(); i++ )
		{
			// Ints can be cast to doubles, so a differing payload type here
			// should not cause the call to return false
			if( type == XmlRpc::XmlRpcValue::TypeDouble &&
				xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) { continue; }
			if( xml[i].getType() != type ) { return false; }
		}
	}
	return true;
}
	
YAML::Node XmlToYaml( XmlRpc::XmlRpcValue& xml )
{
	YAML::Node yaml;
	
	if( xml.getType() != XmlRpc::XmlRpcValue::TypeStruct ) { return yaml; }
	
	XmlRpc::XmlRpcValue::iterator iter;
	for( iter = xml.begin(); iter != xml.end(); iter++ )
	{
		std::string name = iter->first;
		XmlRpc::XmlRpcValue payload = iter->second;
		if( payload.getType() == XmlRpc::XmlRpcValue::TypeStruct )
		{
			yaml[name] = XmlToYaml( payload );
		}
		else if( payload.getType() == XmlRpc::XmlRpcValue::TypeArray )
		{
			if( CheckXmlType( payload, XmlRpc::XmlRpcValue::TypeBoolean ) )
			{
				std::vector<bool> s;
				for( int i = 0; i < payload.size(); i++ )
				{
					s.push_back( static_cast<bool>( payload[i]) );
				}
				yaml[name] = s;
			}
			else if( CheckXmlType( payload, XmlRpc::XmlRpcValue::TypeInt ) )
			{
				std::vector<int> s;
				for( int i = 0; i < payload.size(); i++ )
				{
					s.push_back( static_cast<int>( payload[i]) );
				}
				yaml[name] = s;
			}
			else if( CheckXmlType( payload, XmlRpc::XmlRpcValue::TypeDouble ) )
			{
				std::vector<double> s;
				for( int i = 0; i < payload.size(); i++ )
				{
					
					s.push_back( ParseDouble( payload[i] ) );
				}
				yaml[name] = s;
			}
			else if( CheckXmlType( payload, XmlRpc::XmlRpcValue::TypeString ) )
			{
				std::vector<std::string> s;
				for( int i = 0; i < payload.size(); i++ )
				{
					s.push_back( static_cast<std::string>( payload[i]) );
				}
				yaml[name] = s;
			}
			else
			{
				std::cerr << "Invalid array type." << std::endl;
			}
		}
		else if( payload.getType() == XmlRpc::XmlRpcValue::TypeBoolean )
		{
			yaml[name] = static_cast<bool>( payload );
		}
		else if( payload.getType() == XmlRpc::XmlRpcValue::TypeInt )
		{
			yaml[name] = static_cast<int>( payload );
		}
		else if( payload.getType() == XmlRpc::XmlRpcValue::TypeDouble )
		{
			yaml[name] = static_cast<double>( payload );
		}
		else if( payload.getType() == XmlRpc::XmlRpcValue::TypeString )
		{
			yaml[name] = static_cast<std::string>( payload );
		}
		else
		{
			std::cerr << "Unsupported conversion type." << std::endl;
			continue;
		}
	}
	return yaml;
}

XmlRpc::XmlRpcValue YamlToXml( const YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml;
	
	if( node.IsNull() ) 
	{ 
		return xml; 
	}
	else if( node.IsSequence() )
	{
		std::vector< std::string > contents = node.as< std::vector< std::string > >();
		xml.setSize( contents.size() );
		for( unsigned int i = 0; i < contents.size(); i++ )
		{
			xml[i] = contents[i];
		}
	}
	else if( node.IsScalar() )
	{
		xml = node.as< std::string >();
	}
	else if( node.IsMap() )
	{
		YAML::Node::const_iterator iter;
		for( iter = node.begin(); iter != node.end(); iter++ )
		{
			std::string name = iter->first.as<std::string>();
			xml[ name ] = YamlToXml( iter->second );
		}
	}
	else
	{
		std::cerr << "Invalid YAML node type." << std::endl;
	}
	return xml;
}

void CopyYaml( const YAML::Node& src, YAML::Node& dst )
{
	if( src.IsNull() ) { return; }
	if( src.IsScalar() ) 
	{ 
		dst = src;
		return;
	};
	
	YAML::Node::const_iterator iter;
	for( iter = src.begin(); iter != src.end(); iter++ )
	{
		dst[ iter->first.as<std::string>() ] = iter->second;
	}
}

YAML::Node MergeYaml( const YAML::Node& a, const YAML::Node& b )
{
	// Short circuit cases
	if( a.IsNull() ) { return b; }
	else if( b.IsNull() ) { return a; }
	
	if( !a.IsMap() || !b.IsMap() )
	{
		throw std::runtime_error( "Cannot merge non-map nodes." );
	}
	
	YAML::Node node;
	CopyYaml( a, node );
	YAML::Node::const_iterator iter;
	// Cycle through b and add all fields to node
	for( iter = b.begin(); iter != b.end(); iter++ )
	{
		std::string key = iter->first.as<std::string>();
		
		// If both a and b have a key we have to merge them
		if( node[key] )
		{
			node[key] = MergeYaml( node[key], iter->second );
		}
		// Otherwise we just add it
		else
		{
			node[key] = iter->second;
		}
	}
	return node;
}

bool GetYamlParam( ros::NodeHandle& nh, const std::string name, YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml;
	if( !nh.getParam( name, xml ) ) { return false; }
	node = XmlToYaml( xml );
	return true;
}

YAML::Node SetPoseYaml( const PoseSE3& pose )
{
	YAML::Node node;
	node["orientation"] = SetOrientationYaml( pose.GetQuaternion() );
	node["position"] = SetPositionYaml( pose.GetTranslation() );
	return node;
}

bool GetPoseYaml( const YAML::Node& node, PoseSE3& pose )
{
	if( !node["orientation"] || !node["position"] )
	{
		return false;
	}
	
	Eigen::Quaterniond quat;
	if( !GetOrientationYaml( node["orientation"], quat ) )
	{
		EulerAngles eul;
		if( !GetOrientationYaml( node["orientation"], eul ) ) { return false; }
		quat = EulerToQuaternion( eul );
	}
	
	Eigen::Translation3d pos; 
	if( !GetPositionYaml( node["position"], pos ) ) { return false; }
	pose = PoseSE3( pos, quat );
	return true;
}

YAML::Node SetOrientationYaml( const Eigen::Quaterniond& quat )
{	
	YAML::Node node;
	node["qw"] = quat.w();
	node["qx"] = quat.x();
	node["qy"] = quat.y();
	node["qz"] = quat.z();
	return node;
}

YAML::Node SetOrientationYaml( const EulerAngles& eul )
{
	YAML::Node node;
	node["yaw"] = eul.yaw;
	node["pitch"] = eul.pitch;
	node["roll"] = eul.roll;
	return node;
}

bool GetOrientationYaml( const YAML::Node& node, Eigen::Quaterniond& quat )
{
	if( !node["qw"] || !node["qx"] || !node["qy"] || !node["qz"] ) { return false; }
	
	double qw = node["qw"].as<double>();
	double qx = node["qx"].as<double>();
	double qy = node["qy"].as<double>();
	double qz = node["qz"].as<double>();
	quat = Eigen::Quaterniond( qw, qx, qy, qz );
	return true;
}

bool GetOrientationYaml( const YAML::Node& node, EulerAngles& eul )
{
	if( !node["yaw"] || !node["pitch"] || !node["roll"] ) { return false; }
	
	eul.yaw = node["yaw"].as<double>();
	eul.pitch = node["pitch"].as<double>();
	eul.roll = node["roll"].as<double>();
	return true;
}

YAML::Node SetPositionYaml( const Eigen::Translation3d& trans )
{
	YAML::Node node;
	node["x"] = trans.x();
	node["y"] = trans.y();
	node["z"] = trans.z();
	return node;
}

bool GetPositionYaml( const YAML::Node& node, Eigen::Translation3d& trans )
{
	if( !node["x"] || !node["y"] || !node["z"] ) { return false; }
	
	double x = node["x"].as<double>();
	double y = node["y"].as<double>();
	double z = node["z"].as<double>();
	trans = Eigen::Translation3d( x, y, z );
	return true;
}

YAML::Node SetMatrixYaml( const Eigen::MatrixXd& mat, 
							std::string idDim, std::string idVal )
{
	unsigned int numEls = mat.rows()*mat.cols();
	std::vector<double> vals( numEls );
	std::vector<double> dimensions( 2 );
	dimensions[0] = mat.rows();
	dimensions[1] = mat.cols();
	for( unsigned int i = 0; i < numEls; i++ )
	{
		vals[i] = mat(i);
	}
	YAML::Node node;
	node[idVal] = vals;
	node[idDim] = dimensions;
	return node;
}

bool GetMatrixYaml( const YAML::Node& node, Eigen::MatrixXd& mat, 
					std::string idDim, std::string idVal )
{
	if( !node[idDim] || !node[idVal] ) { return false; }
	std::vector<int> dimensions = node[idDim].as< std::vector<int> >();
	std::vector<double> vals = node[idVal].as< std::vector<double> >();
	if( dimensions.size() != 2 )
	{
		std::cerr << "Invalid matrix dimension field." << std::endl;
		return false;
	}
	
	if( dimensions[0]*dimensions[1] != vals.size() )
	{
		std::cerr << "Invalid number of values for matrix." << std::endl;
		return false;
	}
	
	mat = Eigen::MatrixXd( dimensions[0], dimensions[1] );
	return ParseMatrix( vals, mat, RowMajor );

}
	
} // end namespace argus_utils

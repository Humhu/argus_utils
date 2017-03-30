#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
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

YAML::Node SetPoseYaml( const PoseSE3& pose )
{
	YAML::Node ori = SetOrientationYaml( pose.GetQuaternion() );
	YAML::Node pos = SetPositionYaml( pose.GetTranslation() );
	return MergeYaml(pos, ori);
}

// bool GetPoseYaml( const YAML::Node& node, PoseSE3& pose )
// {
// 	if( !node["orientation"] || !node["position"] )
// 	{
// 		return false;
// 	}
	
// 	Eigen::Quaterniond quat;
// 	if( !GetOrientationYaml( node["orientation"], quat ) )
// 	{
// 		EulerAngles eul;
// 		if( !GetOrientationYaml( node["orientation"], eul ) ) { return false; }
// 		quat = EulerToQuaternion( eul );
// 	}
	
// 	Eigen::Translation3d pos; 
// 	if( !GetPositionYaml( node["position"], pos ) ) { return false; }
// 	pose = PoseSE3( pos, quat );
// 	return true;
// }

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

YAML::Node SetPositionYaml( const Eigen::Translation3d& trans )
{
	YAML::Node node;
	node["x"] = trans.x();
	node["y"] = trans.y();
	node["z"] = trans.z();
	return node;
}

YAML::Node SetMatrixYaml( const MatrixType& mat )
{
	YAML::Node node;
	node["rows"] = mat.rows();
	node["cols"] = mat.cols();
	node["data"] = std::vector<double>( mat.data(), mat.data() + mat.size() );
	node["column_major"] = true;
	return node;
}

bool GetMatrixYaml( const YAML::Node& node, MatrixType& mat )
{
	unsigned int rows, cols;
	std::vector<double> values;
	bool colMajor;
	if( !GetParam( node, "column_major", colMajor )
	 || !GetParam( node, "rows", rows )
	 || !GetParam( node, "cols", cols )
	 || !GetParam( node, "values", values ) ) { return false; }

	if( colMajor )
	{
		mat = Eigen::Map<ColMatrixType>( values.data(), rows, cols );
	}
	else
	{
		mat = Eigen::Map<RowMatrixType>( values.data(), rows, cols );
	}
	return true;
}
	
} // end namespace argus

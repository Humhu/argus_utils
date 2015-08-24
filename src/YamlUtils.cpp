#include "argus_utils/YamlUtils.h"

namespace argus_utils
{

std::string ReplaceAll( std::string input, const std::string query, const std::string rep )
{
	size_t loc;
	while( (loc = input.find( query )) != std::string::npos )
	{
		input.replace( loc, rep.size(), rep );
	}
	return input;
}

bool CheckXmlType( const XmlRpc::XmlRpcValue& xml, XmlRpc::XmlRpcValue::Type type )
{
	if( xml.getType() == XmlRpc::XmlRpcValue::TypeArray )
	{
		for( unsigned int i = 0; i < xml.size(); i++ )
		{
			if( xml[i].getType() != type ) { return false; }
		}
	}
	return true;
}
	
YAML::Node XmlToYaml( XmlRpc::XmlRpcValue& xml )
{
	YAML::Node yaml;
	
	if( xml.getType() == XmlRpc::XmlRpcValue::TypeInvalid ) { return yaml; }
	
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
					s.push_back( static_cast<double>( payload[i]) );
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
			yaml[name] = static_cast<bool>( payload );;
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

YAML::Node SetPoseYaml( const PoseSE3& pose )
{
	YAML::Node node;
	node["quaternion"] = SetQuaternionYaml( pose.GetQuaternion() );
	node["position"] = SetPositionYaml( pose.GetTranslation() );
	return node;
}

bool GetPoseYaml( const YAML::Node& node, PoseSE3& pose )
{
	if( !node["quaternion"] || !node["position"] )
	{
		throw std::runtime_error( "Missing quaternion/position field from pose." );
	}
	Eigen::Quaterniond quat; 
	if( !GetQuaternionYaml( node["quaternion"], quat ) ) { return false; }
	Eigen::Translation3d pos; 
	if( !GetPositionYaml( node["position"], pos ) ) { return false; }
	pose = PoseSE3( pos, quat );
	return true;
}

YAML::Node SetQuaternionYaml( const Eigen::Quaterniond& quat )
{
	std::vector<double> vals(4);
	vals[0] = quat.w();
	vals[1] = quat.x();
	vals[2] = quat.y();
	vals[3] = quat.z();
	YAML::Node node;
	node = vals;
	return node;
}

bool GetQuaternionYaml( const YAML::Node& node, Eigen::Quaterniond& quat )
{
	if( !node.IsSequence() )
	{
		throw std::runtime_error( "Null node in GetQuaternion!" );
	}
	
	std::vector<double> vals;
	try
	{
		vals = node.as< std::vector<double> >();
	}
	catch( std::exception e )
	{
		std::stringstream ss;
		ss << "Error parsing quaternion string: " << node;
		throw std::runtime_error( ss.str() );
	}
	
	if( vals.size() != 4 )
	{
		throw std::runtime_error( "Incorrect number of elements for quaternion." );
	}
	quat = Eigen::Quaterniond( vals[0], vals[1], vals[2], vals[3] );
	return true;
}

YAML::Node SetPositionYaml( const Eigen::Translation3d& trans )
{
	std::vector<double> vals(3);
	vals[0] = trans.x();
	vals[1] = trans.y();
	vals[2] = trans.z();
	YAML::Node node;
	node = vals;
	return node;
}

bool GetPositionYaml( const YAML::Node& node, Eigen::Translation3d& trans )
{
	std::vector<double> vals = node.as< std::vector<double> >();
	if( vals.size() != 3 )
	{
		throw std::runtime_error( "Incorrect number of elements for position." );
	}
	Eigen::Vector3d vec( vals[0], vals[1], vals[2] );
	trans = Eigen::Translation3d( vec );
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
		throw std::runtime_error( "Invalid matrix dimension field." );
	}
	
	if( dimensions[0]*dimensions[1] != vals.size() )
	{
		throw std::runtime_error( "Invalid number of values for matrix." );
	}
	
	mat = Eigen::MatrixXd( dimensions[0], dimensions[1] );
	for( unsigned int i = 0; i < dimensions[0]*dimensions[1]; i++ )
	{
		mat(i) = vals[i];
	}
	return false;
}
	
} // end namespace argus_utils

#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include "argus_utils/PoseSE3.h"
#include "argus_utils/GeometryUtils.h"

namespace argus_utils
{
	
/*! \brief Convert to/from the XmlRpc type used by ROS. */
// NOTE XmlRpc is awful to use and doesn't have a const iterator
// TODO Robustify this function. It still fails on some instances
YAML::Node XmlToYaml( XmlRpc::XmlRpcValue& xml );
XmlRpc::XmlRpcValue YamlToXml( const YAML::Node& node );

/*! \brief Get a parameter YAML object out of a node handle by combining calls
 * to get XmlRpc and convert it. Returns success. */
bool GetYamlParam( ros::NodeHandle& nh, const std::string name, YAML::Node& node );

/*! \brief Read/Write a PoseSE3 object to the node */
YAML::Node SetPoseYaml( const PoseSE3& pose );
bool GetPoseYaml( const YAML::Node& node, PoseSE3& pose );

/*! \brief Populate a pose message from a YAML block */
// TODO geometry_msgs::Pose ParsePoseYaml( const YAML::Node& node );

/*! \brief Read/Write a Quaternion object to the node */
YAML::Node SetQuaternionYaml( const Eigen::Quaterniond& quat );
bool GetQuaternionYaml( const YAML::Node& node, Eigen::Quaterniond& quat );

bool GetEulerYaml( const YAML::Node& node, EulerAngles& eul );

/*! \brief Read/Write a Position object to the node */
YAML::Node SetPositionYaml( const Eigen::Translation3d& trans );
bool GetPositionYaml( const YAML::Node& node, Eigen::Translation3d& trans );

/*! \brief Read/Write a matrix to the node */
YAML::Node SetMatrixYaml( const Eigen::MatrixXd& mat, 
						  std::string idDim = "dimensions", std::string idVal = "values" );
bool GetMatrixYaml( const YAML::Node& node, Eigen::MatrixXd& mat, 
					std::string idDim = "dimensions", std::string idVal = "values" );
	
} // end namespace argus_utils

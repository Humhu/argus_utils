#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/GeometryUtils.h"

namespace argus
{

/*! \brief Convert to/from the XmlRpc type used by ROS. */
// NOTE XmlRpc is awful to use and doesn't have a const iterator
// TODO Robustify this function. It still fails on some instances
YAML::Node XmlToYaml( XmlRpc::XmlRpcValue& xml );
XmlRpc::XmlRpcValue YamlToXml( const YAML::Node& node );

/*! \brief Copies one YAML node to another. Avoids lazy copying. */
void CopyYaml( const YAML::Node& src, YAML::Node& dst );

/*! \brief Merges two YAML nodes into one with a union operation. */ 
YAML::Node MergeYaml( const YAML::Node& a, const YAML::Node& b );

/*! \brief Read/Write a PoseSE3 object to the node */
YAML::Node SetPoseYaml( const PoseSE3& pose );

// TODO Support float and double types
/*! \brief Read/Write a quaternion or Euler object to the node */
YAML::Node SetOrientationYaml( const Eigen::Quaterniond& quat );
YAML::Node SetOrientationYaml( const EulerAngles& eul );

/*! \brief Read/Write a Position object to the node */
YAML::Node SetPositionYaml( const Eigen::Translation3d& trans );

// TODO versions for column major?
YAML::Node SetMatrixYaml( const MatrixType& mat );
bool GetMatrixYaml( const YAML::Node& node, MatrixType& mat );
	
} // end namespace argus

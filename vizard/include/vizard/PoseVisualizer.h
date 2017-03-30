#pragma once

#include "vizard/Visualizer.h"
#include "std_msgs/ColorRGBA.h"

namespace argus
{

/*! \brief Visualizes pose messages as colored tri-axes RGB-XYZ */
class PoseVisualizer : public Visualizer
{
public:

	PoseVisualizer();

	void ReadParams( const ros::NodeHandle& nh );

	void SetLinewidth( double a );
	void SetAxesLength( double l );

	std::vector<MarkerMsg> ToMarkers( const PoseSE3& pose,
	                                  const std::string& name = "" ) const;
	std::vector<MarkerMsg> ToMarkers( const std::vector<PoseSE3>& pose,
	                                  const std::vector<std::string>& names = {} ) const;

private:

	double _linewidth;
	double _axesLength;

	void SetColors( std_msgs::ColorRGBA& xColor,
	                std_msgs::ColorRGBA& yColor,
	                std_msgs::ColorRGBA& zColor ) const;
	void AddPosePoints( const PoseSE3& pose,
	                    std::vector<geometry_msgs::Point>& xPoints,
	                    std::vector<geometry_msgs::Point>& yPoints,
	                    std::vector<geometry_msgs::Point>& zPoints ) const;
};

}
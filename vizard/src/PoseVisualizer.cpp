#include "vizard/PoseVisualizer.h"

#include "argus_utils/utils/ParamUtils.h"

#include "geometry_msgs/Point.h"

namespace argus
{

geometry_msgs::Point vec3_to_point( const Eigen::Vector3d& vec )
{
	geometry_msgs::Point p;
	p.x = vec( 0 );
	p.y = vec( 1 );
	p.z = vec( 2 );
	return p;
}

PoseVisualizer::PoseVisualizer()
{
	SetLinewidth( 1.0 );
	SetAlpha( 1.0 );
	SetAxesLength( 1.0 );
}

void PoseVisualizer::ReadParams( const ros::NodeHandle& nh )
{
	Visualizer::ReadParams( nh );
	double a;
	if( GetParam( nh, "linewidth", a ) ) { SetLinewidth( a ); }
	if( GetParam( nh, "axes_length", a ) ) { SetAxesLength( a ); }
}

void PoseVisualizer::SetLinewidth( double a )
{
	if( a < 0 )
	{
		throw std::invalid_argument( "Linewidth must be positive" );
	}
	_linewidth = a;
}

void PoseVisualizer::SetAxesLength( double l )
{
	if( l < 0 )
	{
		throw std::invalid_argument( "Axes length must be positive" );
	}
	_axesLength = l;
}

std::vector<MarkerMsg> PoseVisualizer::ToMarkers( const PoseSE3& pose,
                                                  const std::string& name ) const
{
	MarkerMsg xMarker = InitMarker();
	MarkerMsg yMarker = InitMarker();
	MarkerMsg zMarker = InitMarker();

	xMarker.id = VisualizerID::POSE_X_ID;
	yMarker.id = VisualizerID::POSE_Y_ID;
	zMarker.id = VisualizerID::POSE_Z_ID;

	xMarker.type = MarkerMsg::LINE_LIST;
	yMarker.type = MarkerMsg::LINE_LIST;
	zMarker.type = MarkerMsg::LINE_LIST;
	xMarker.scale.x = _linewidth;
	yMarker.scale.x = _linewidth;
	zMarker.scale.x = _linewidth;

	SetColors( xMarker.color, yMarker.color, zMarker.color );
	AddPosePoints( pose, xMarker.points, yMarker.points, zMarker.points );

	std::vector<MarkerMsg> markers;
	markers.push_back( xMarker );
	markers.push_back( yMarker );
	markers.push_back( zMarker );
	AddNameMarker( name, pose, markers );

	return markers;
}

std::vector<MarkerMsg> PoseVisualizer::ToMarkers( const std::vector<PoseSE3>& poses,
                                                  const std::vector<std::string>& names ) const
{
	if( !names.empty() && names.size() != poses.size() )
	{
		throw std::invalid_argument( "Must specify equal number of names and poses." );
	}

	std::vector<MarkerMsg> markers;
	if( poses.empty() ) { return markers; }

	MarkerMsg xMarker = InitMarker();
	MarkerMsg yMarker = InitMarker();
	MarkerMsg zMarker = InitMarker();

	xMarker.id = VisualizerID::POSE_X_ID;
	yMarker.id = VisualizerID::POSE_Y_ID;
	zMarker.id = VisualizerID::POSE_Z_ID;

	xMarker.type = MarkerMsg::LINE_LIST;
	yMarker.type = MarkerMsg::LINE_LIST;
	zMarker.type = MarkerMsg::LINE_LIST;
	xMarker.scale.x = _linewidth;
	yMarker.scale.x = _linewidth;
	zMarker.scale.x = _linewidth;

	SetColors( xMarker.color, yMarker.color, zMarker.color );

	for( unsigned int i = 0; i < poses.size(); ++i )
	{
		const PoseSE3& pose = poses[i];
		AddPosePoints( pose, xMarker.points, yMarker.points, zMarker.points );

		if( !names.empty() )
		{
			const std::string& name = names[i];
			AddNameMarker( name, pose, markers );
		}
	}

	markers.push_back( xMarker );
	markers.push_back( yMarker );
	markers.push_back( zMarker );
	return markers;
}

void PoseVisualizer::SetColors( std_msgs::ColorRGBA& xColor,
                                std_msgs::ColorRGBA& yColor,
                                std_msgs::ColorRGBA& zColor ) const
{
	xColor.r = 1; xColor.g = 0; xColor.b = 0;
	yColor.r = 0; yColor.g = 1; yColor.b = 0;
	zColor.r = 0; zColor.g = 0; zColor.b = 1;
	xColor.a = _alpha;
	yColor.a = _alpha;
	zColor.a = _alpha;
}

void PoseVisualizer::AddPosePoints( const PoseSE3& pose,
                                    std::vector<geometry_msgs::Point>& xPoints,
                                    std::vector<geometry_msgs::Point>& yPoints,
                                    std::vector<geometry_msgs::Point>& zPoints ) const
{
	PoseSE3::Transform trans = pose.ToTransform();
	geometry_msgs::Point zero = vec3_to_point( trans * Eigen::Vector3d::Zero() );
	Eigen::Vector3d xTrans = trans * ( _axesLength * Eigen::Vector3d::UnitX() );
	Eigen::Vector3d yTrans = trans * ( _axesLength * Eigen::Vector3d::UnitY() );
	Eigen::Vector3d zTrans = trans * ( _axesLength * Eigen::Vector3d::UnitZ() );
	xPoints.push_back( zero );
	xPoints.push_back( vec3_to_point( xTrans ) );
	yPoints.push_back( zero );
	yPoints.push_back( vec3_to_point( yTrans ) );
	zPoints.push_back( zero );
	zPoints.push_back( vec3_to_point( zTrans ) );
}

}
#include "vizard/Visualizer.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

Visualizer::Visualizer() 
{
    SetAlpha( 1.0 );
    SetColor( 1.0, 1.0, 1.0 );
    
    SetTextAlpha( 1.0 );
    SetTextColor( 1.0, 1.0, 1.0 );
    SetTextOffset( PoseSE3( 0, 0, -0.1, 1, 0, 0, 0 ) );
    SetTextSize( 0.1 );
    SetTextUniqueNames( false );
    
    SetFrameID( "" );
    SetMarkerName( "marker" );
    SetShowName( false );
}

void Visualizer::ReadParams( const ros::NodeHandle& nh )
{
    double a;
    double r, g, b;
    PoseSE3 off;
    std::string s;
    bool t;

    if( GetParam( nh, "alpha", a ) ) { SetAlpha( a ); }
    if( GetParam( nh, "red", r ) &&
        GetParam( nh, "green", g ) &&
        GetParam( nh, "blue", b ) ) { SetColor( r, g, b ); }
    
    if( GetParam( nh, "text_alpha", a ) ) { SetTextAlpha( a ); }
    if( GetParam( nh, "text_red", r ) &&
        GetParam( nh, "text_green", g ) &&
        GetParam( nh, "text_blue", b ) ) { SetTextColor( r, g, b ); }
    if( GetParam( nh, "text_offset", off ) ) { SetTextOffset( off ); }
    if( GetParam( nh, "text_size", a ) ) { SetTextSize( a ); }
    if( GetParam( nh, "text_unique_names", t ) ) { SetTextUniqueNames( t ); }

    if( GetParam( nh, "frame_id", s ) ) { SetFrameID( s ); }
    if( GetParam( nh, "marker_name", s ) ) { SetMarkerName( s ); }
    if( GetParam( nh, "show_name", t ) ) { SetShowName( t ); }
}

void Visualizer::SetAlpha( double a )
{
    if( a < 0 || a > 1.0 )
    {
        throw std::invalid_argument( "Alpha must be between 0 and 1." );
    }
    _alpha = a;
}

void Visualizer::SetColor( double r, double g, double b )
{
    if( r < 0 || r > 1 || g < 0 || g > 1 || b < 0 || b > 1 )
    {
        throw std::invalid_argument( "Colors must be between 0 and 1" );
    }
    _r = r;
    _g = g;
    _b = b;
}

void Visualizer::SetTextAlpha( double a )
{
    if( a < 0 || a > 1.0 )
    {
        throw std::invalid_argument( "Alpha must be between 0 and 1." );
    }
    _alphaText = a;
}

void Visualizer::SetTextColor( double r, double g, double b )
{
    if( r < 0 || r > 1 || g < 0 || g > 1 || b < 0 || b > 1 )
    {
        throw std::invalid_argument( "Colors must be between 0 and 1" );
    }
    _rText = r;
    _gText = g;
    _bText = b;
}

void Visualizer::SetTextOffset( const PoseSE3& off )
{
    _offsetText = off;
}

void Visualizer::SetTextSize( double h )
{
    if( h < 0 )
    {
        throw std::invalid_argument( "Text size must be positive." );
    }
    _hText = h;
}

void Visualizer::SetTextUniqueNames( bool e )
{
    _uniqueTextNames = e;
}


void Visualizer::SetFrameID( const std::string& id )
{
    _frameID = id;
}

void Visualizer::SetMarkerName( const std::string& n )
{
    _markerName = n;
}

void Visualizer::SetShowName( bool e )
{
    _showText = e;
}

MarkerMsg Visualizer::InitMarker() const
{
    MarkerMsg marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = _frameID;
    marker.action = MarkerMsg::ADD;
    marker.ns = _markerName;
    marker.color.r = _r;
    marker.color.g = _g;
    marker.color.b = _b;
    marker.color.a = _alpha;
    return marker;
}

void Visualizer::AddNameMarker( const std::string& name,
                                const PoseSE3& pose,
                                std::vector<MarkerMsg>& markers ) const
{
    if( !_showText || name.empty() ) { return; }

    MarkerMsg marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = _frameID;
    marker.action = MarkerMsg::ADD;
    marker.ns = _markerName;
    marker.id = VisualizerID::LABEL_ID;
    marker.type = MarkerMsg::TEXT_VIEW_FACING;
    marker.color.r = _rText;
    marker.color.g = _gText;
    marker.color.b = _bText;
    marker.color.a = _alphaText;
    marker.text = name;
    marker.scale.z = _hText;
    marker.pose = PoseToMsg( pose * _offsetText );
    markers.push_back( marker );
}

}
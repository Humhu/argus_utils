#pragma once

#include <ros/ros.h>

#include "visualization_msgs/Marker.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/GeometryTypes.h"

namespace argus
{

enum VisualizerID
{
    POSE_X_ID = 0,
    POSE_Y_ID,
    POSE_Z_ID,
    LABEL_ID,
    FID_ID
};

typedef visualization_msgs::Marker MarkerMsg;

/*! \brief Base class for all visualizers. */
class Visualizer
{
public:

    Visualizer();
    void ReadParams( const ros::NodeHandle& nh );

    virtual void SetColor( double r, double g, double b );
    virtual void SetAlpha( double a );

    virtual void SetTextColor( double r, double g, double b );
    virtual void SetTextAlpha( double a );
    virtual void SetTextOffset( const PoseSE3& off );
    virtual void SetTextSize( double h );
    
    virtual void SetFrameID( const std::string& id );
    virtual void SetMarkerName( const std::string& n );
    virtual void SetShowName( bool e );

    // TODO
    //virtual std::vector<MarkerMsg> ToMarkers() = 0;

protected:

    double _r;
    double _g;
    double _b;
    double _alpha;

    double _rText;
    double _gText;
    double _bText;
    double _alphaText;
    double _hText;
    PoseSE3 _offsetText;

    std::string _markerName;
    std::string _frameID;
    bool _showText;

    MarkerMsg InitMarker() const;
    void AddNameMarker( const std::string& name,
                        const PoseSE3& pose,
                        std::vector<MarkerMsg>& markers ) const;
};


}
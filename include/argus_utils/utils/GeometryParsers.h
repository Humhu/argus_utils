#pragma once

#include "argus_utils/utils/CoreParsers.h"
#include "argus_utils/geometry/GeometryTypes.h"
#include "argus_utils/geometry/GeometryUtils.h"

namespace argus
{

template <typename S>
bool GetParam( const S& nh, const std::string& name, QuaternionType& quat )
{
	YAML::Node node;
	if( !GetParam( nh, name, node ) ) { return false; }
	
	double w, x, y, z;
	if( !GetParam( node, "qx", x ) ||
	    !GetParam( node, "qy", y ) ||
	    !GetParam( node, "qz", z ) ||
	    !GetParam( node, "qw", w ) ) { return false; }

	quat = QuaternionType( w, x, y, z );
	return true;
}

template <typename S>
bool GetParam( const S& nh, const std::string& name, EulerAngles& eul )
{
	YAML::Node node;
	if( !GetParam( nh, name, node ) ) { return false; }

	if( !GetParam( node, "yaw", eul.yaw ) ||
	    !GetParam( node, "pitch", eul.pitch ) ||
	    !GetParam( node, "roll", eul.roll ) ) { return false; }
	return true;
}

template <typename S>
bool GetParam( const S& nh, const std::string& name, Translation3Type& trans )
{
	YAML::Node node;
	if( !GetParam( nh, name, node ) ) { return false; }
	
	double x, y, z;
	if( !GetParam( node, "x", x ) ||
	    !GetParam( node, "y", y ) ||
	    !GetParam( node, "z", z ) ) { return false; }

	trans = Translation3Type( x, y, z );
	return true;
}

template <typename S>
bool GetParam( const S& nh, const std::string& name, PoseSE3& pose )
{
	QuaternionType quat;
	Translation3Type trans;
	if( !GetParam( nh, name, trans ) ) { return false; }
	if( !GetParam( nh, name, quat ) )
	{
		EulerAngles eul;
		if( !GetParam( nh, name, eul ) ) { return false; }
		quat = EulerToQuaternion( eul );
	}
	pose = PoseSE3( trans, quat );
	return true;
}

}
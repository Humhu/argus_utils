#include "argus_utils/filters/FilterUtils.h"

namespace argus
{

PredictInfo MsgToPredict( const argus_msgs::FilterStepInfo& msg )
{
	if( !msg.isPredict )
	{
		throw std::runtime_error( "Cannot parse update msg to predict." );
	}
	PredictInfo info;
	info.Spre = MsgToMatrix( msg.Spre );
	info.dt = msg.dt;
	info.F = MsgToMatrix( msg.F );
	return info;
}

argus_msgs::FilterStepInfo PredictToMsg( const PredictInfo& info )
{
	argus_msgs::FilterStepInfo msg;
	msg.Spre = MatrixToMsg( info.Spre );
	msg.dt = info.dt;
	msg.F = MatrixToMsg( info.F );
	msg.isPredict = true;
	return msg;
}

UpdateInfo MsgToUpdate( const argus_msgs::FilterStepInfo& msg )
{
	if( msg.isPredict )
	{
		throw std::runtime_error( "Cannot parse predict msg to update." );
	}
	UpdateInfo info;
	info.Spre = MsgToMatrix( msg.Spre );
	info.H = MsgToMatrix( msg.H );

	// NOTE Must initialize size or else parsing fails
	// TODO Update this parse function interface to avoid these kinds of errors in the future
	info.innovation = VectorType( info.H.rows() );
	ParseMatrix( msg.innovation, info.innovation );
	return info;
}

argus_msgs::FilterStepInfo UpdateToMsg( const UpdateInfo& info )
{
	argus_msgs::FilterStepInfo msg;
	msg.Spre = MatrixToMsg( info.Spre );
	SerializeMatrix( info.innovation, msg.innovation );
	msg.H = MatrixToMsg( info.H );
	msg.isPredict = false;
	return msg;
}

}
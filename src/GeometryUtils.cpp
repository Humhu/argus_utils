#include "argus_utils/geometry/GeometryUtils.h"

namespace argus
{
	
EulerAngles QuaternionToEuler( const QuaternionType& quat )
{
	double temp1, temp2;
	double psi, theta, phi;
	
	temp1 = quat.w()*quat.y() - quat.z()*quat.x();
	if(temp1 > 0.999) 
	{
		psi = 2*atan2(quat.w(), quat.x());
		theta = M_PI/2;
		phi = 0;
	} 
	else if(temp1 < -0.999) 
	{
		psi = -2*atan2(quat.w(), quat.x());
		theta = -M_PI/2;
		phi = 0;
	} 
	else 
	{
		theta = asin(2.0*temp1);
		temp1 = 2.0*(quat.w()*quat.x() + quat.y()*quat.z());
		temp2 = 1.0 - 2.0*(quat.x()*quat.x() + quat.y()*quat.y());
		phi = atan2(temp1, temp2);
		temp1 = 2.0*(quat.w()*quat.z() + quat.x()*quat.y());
		temp2 = 1.0 - 2.0*(quat.y()*quat.y() + quat.z()*quat.z());
		psi = atan2(temp1, temp2);
	}

	EulerAngles e;
	e.yaw = psi;
	e.pitch = theta;
	e.roll = phi;
	return e;
}

QuaternionType EulerToQuaternion( const EulerAngles& eul )
{
	return QuaternionType( Eigen::AngleAxisd(eul.yaw, Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(eul.pitch, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(eul.roll, Eigen::Vector3d::UnitX()) );
}

geometry_msgs::Vector3 PositionToMsg( const Translation3Type& trans )
{
	geometry_msgs::Vector3 msg;
	msg.x = trans.x();
	msg.y = trans.y();
	msg.z = trans.z();
	return msg;
}

Translation3Type MsgToPosition( const geometry_msgs::Vector3& msg )
{
	return Translation3Type( msg.x, msg.y, msg.z );
}

QuaternionType MsgToQuaternion( const geometry_msgs::Quaternion& msg )
{
	return QuaternionType( msg.w, msg.x, msg.y, msg.z );
}

geometry_msgs::Quaternion QuaternionToMsg( const QuaternionType& quat )
{
	geometry_msgs::Quaternion msg;
	msg.x = quat.x();
	msg.y = quat.y();
	msg.z = quat.z();
	msg.w = quat.w();
	return msg;
}

PoseSE3 TransformToPose( const geometry_msgs::Transform& msg )
{
	return PoseSE3( MsgToPosition( msg.translation ),
	                MsgToQuaternion( msg.rotation ) );
}

geometry_msgs::Transform PoseToTransform( const PoseSE3& pose )
{
	geometry_msgs::Transform msg;
	msg.translation = PositionToMsg( pose.GetTranslation() );
	msg.rotation = QuaternionToMsg( pose.GetQuaternion() );
	return msg;
}

geometry_msgs::Pose PoseToMsg( const PoseSE2& pose )
{
	return PoseToMsg( PoseSE3::FromSE2( pose ) );
}

geometry_msgs::Pose PoseToMsg( const PoseSE3& pose )
{
	geometry_msgs::Pose msg;
	
	Translation3Type trans = pose.GetTranslation();
	msg.position.x = trans.x();
	msg.position.y = trans.y();
	msg.position.z = trans.z();
	msg.orientation = QuaternionToMsg( pose.GetQuaternion() );
	
	return msg;
}

PoseSE3 MsgToPose( const geometry_msgs::Pose& msg )
{
	Translation3Type trans( msg.position.x, msg.position.y, msg.position.z );
	QuaternionType quat = MsgToQuaternion( msg.orientation );
	return PoseSE3( trans, quat );
}

geometry_msgs::Twist TangentToMsg( const PoseSE2::TangentVector& tan )
{
	geometry_msgs::Twist twist;
	twist.linear.x = tan(0);
	twist.linear.y = tan(1);
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = tan(2);
	return twist;
}

geometry_msgs::Twist TangentToMsg( const PoseSE3::TangentVector& tan )
{
	geometry_msgs::Twist twist;
	twist.linear.x = tan(0);
	twist.linear.y = tan(1);
	twist.linear.z = tan(2);
	twist.angular.x = tan(3);
	twist.angular.y = tan(4);
	twist.angular.z = tan(5);
	return twist;
}

PoseSE3::TangentVector MsgToTangent( const geometry_msgs::Twist& twist )
{
	PoseSE3::TangentVector tan;
	tan(0) = twist.linear.x;
	tan(1) = twist.linear.y;
	tan(2) = twist.linear.z;
	tan(3) = twist.angular.x;
	tan(4) = twist.angular.y;
	tan(5) = twist.angular.z;
	return tan;
}

FixedVectorType<3> MsgToVector3( const geometry_msgs::Vector3& msg )
{
	FixedVectorType<3> vec;
	vec(0) = msg.x;
	vec(1) = msg.y;
	vec(2) = msg.z;
	return vec;
}

std::ostream& operator<<( std::ostream& os, const EulerAngles& eul ) 
{
	os << "Y: " << eul.yaw << " P: " << eul.pitch << " R: " << eul.roll;
	return os;
}

} // end namespace argus

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

PoseSE3 TfToPose( const tf::Transform& trans )
{
	tf::Vector3 translation = trans.getOrigin();
	tf::Quaternion quat = trans.getRotation();
	return PoseSE3( translation.getX(), translation.getY(), translation.getZ(),
	                quat.w(), quat.x(), quat.y(), quat.z() );
}

tf::Transform PoseToTf( const PoseSE3& pose )
{
	QuaternionType quat = pose.GetQuaternion();
	tf::Quaternion tfQuat( quat.x(), quat.y(), quat.z(), quat.w() );
	Translation3Type trans = pose.GetTranslation();
	tf::Vector3 tfTrans( trans.x(), trans.y(), trans.z() );
	return tf::Transform( tfQuat, tfTrans );
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

std::ostream& operator<<( std::ostream& os, const EulerAngles& eul ) 
{
	os << "Y: " << eul.yaw << " P: " << eul.pitch << " R: " << eul.roll;
	return os;
}

} // end namespace argus

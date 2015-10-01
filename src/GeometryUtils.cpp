#include "argus_utils/GeometryUtils.h"

namespace argus_utils
{

EulerAngles::EulerAngles( double y, double p, double r )
	: yaw( y ), pitch( p ), roll( r ) {}
	
EulerAngles QuaternionToEuler( const Eigen::Quaterniond& quat )
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

Eigen::Quaterniond EulerToQuaternion( const EulerAngles& eul )
{
	return Eigen::Quaterniond( Eigen::AngleAxisd(eul.yaw, Eigen::Vector3d::UnitZ())
							   * Eigen::AngleAxisd(eul.pitch, Eigen::Vector3d::UnitY())
							   * Eigen::AngleAxisd(eul.roll, Eigen::Vector3d::UnitX()) );
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
	PoseSE3::Quaternion quat = pose.GetQuaternion();
	tf::Quaternion tfQuat( quat.x(), quat.y(), quat.z(), quat.w() );
	PoseSE3::Translation trans = pose.GetTranslation();
	tf::Vector3 tfTrans( trans.x(), trans.y(), trans.z() );
	return tf::Transform( tfQuat, tfTrans );
}

geometry_msgs::Pose PoseToMsg( const PoseSE3& pose )
{
	geometry_msgs::Pose msg;
	
	PoseSE3::Translation trans = pose.GetTranslation();
	msg.position.x = trans.x();
	msg.position.y = trans.y();
	msg.position.z = trans.z();
	PoseSE3::Quaternion quat = pose.GetQuaternion();
	msg.orientation.x = quat.x();
	msg.orientation.y = quat.y();
	msg.orientation.z = quat.z();
	msg.orientation.w = quat.w();
	
	return msg;
}

PoseSE3 MsgToPose( const geometry_msgs::Pose& msg )
{
	PoseSE3::Translation trans( msg.position.x, msg.position.y, msg.position.z );
	PoseSE3::Quaternion quat( msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z );
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

} // end namespace argus_utils

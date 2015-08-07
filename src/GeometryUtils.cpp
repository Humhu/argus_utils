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

tf::Transform PoseToTf( const PoseSE3& pose )
{
	PoseSE3::Quaternion quat = pose.GetQuaternion();
	tf::Quaternion tfQuat( quat.x(), quat.y(), quat.z(), quat.w() );
	PoseSE3::Translation trans = pose.GetTranslation();
	tf::Vector3 tfTrans( trans.x(), trans.y(), trans.z() );
	return tf::Transform( tfQuat, tfTrans );
}

std::ostream& operator<<( std::ostream& os, const EulerAngles& eul ) 
{
	os << "Y: " << eul.yaw << " P: " << eul.pitch << " R: " << eul.roll;
	return os;
}

} // end namespace argus_utils

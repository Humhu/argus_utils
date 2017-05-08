#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include <cmath>

namespace argus 
{

const int PoseSE3::VectorDimension;
const int PoseSE3::TangentDimension;

PoseSE3::PoseSE3() {}
	
PoseSE3::PoseSE3( double x, double y, double z, double qw, 
                  double qx, double qy, double qz )
{
	QuaternionType q( qw, qx, qy, qz );
	q.normalize();
	Sophus::SE3d::Point t;
	t << x, y, z;
	_tform = Sophus::SE3d( q, t );
}

PoseSE3::PoseSE3( const VectorType& vec ) 
{
	QuaternionType q( vec(3), vec(4), vec(5), vec(6) );
	q.normalize();
	_tform = Sophus::SE3d( q, vec.head<3>() );
}

PoseSE3::PoseSE3( const Transform& t ) 
: _tform( t.rotation(), t.translation() ) 
{}

PoseSE3::PoseSE3( const MatrixType& mat ) 
{
	if( mat.rows() == 3 && mat.cols() == 3 )
	{
		Sophus::SE3d::Point t;
		t << 0, 0, 0;
		_tform = Sophus::SE3d( FixedMatrixType<3,3>( mat ), t );
	}
	if( mat.rows() == 4 || mat.cols() == 4 )
	{
		_tform = Sophus::SE3d( FixedMatrixType<4,4>( mat ) );
	}
	else
	{
		throw std::runtime_error( "PoseSE3: Invalid matrix input." );
	}
}

PoseSE3::PoseSE3( const FixedMatrixType<4,4>& mat ) 
: _tform( mat ) 
{}

PoseSE3::PoseSE3( const FixedMatrixType<3,3>& rot )
{
	Sophus::SE3d::Point t;
	t << 0, 0, 0;
	_tform = Sophus::SE3d( rot, t );
}

PoseSE3::PoseSE3( const Translation3Type& t, const QuaternionType& q ) 
{
	Sophus::SE3d::Point p;
	p << t.x(), t.y(), t.z();
	QuaternionType quat = q;
	quat.normalize();
	_tform = Sophus::SE3d( quat, p );
}

PoseSE3 PoseSE3::FromSE2( const PoseSE2& se2 ) 
{
	MatrixType H = se2.ToTransform().matrix();
	MatrixType H3 = MatrixType::Identity(4,4);
	H3.topLeftCorner<2,2>() = H.topLeftCorner<2,2>();
	H3.topRightCorner<2,1>() = H.topRightCorner<2,1>();

	return PoseSE3( H3 );
}

FixedMatrixType<4,4> PoseSE3::ToMatrix() const
{
	MatrixType H = _tform.matrix();
	return H;
}

Translation3Type PoseSE3::GetTranslation() const 
{
	return Translation3Type( _tform.translation() );
}

QuaternionType PoseSE3::GetQuaternion() const 
{
	return _tform.unit_quaternion();
}

FixedVectorType<PoseSE3::VectorDimension> PoseSE3::ToVector() const 
{
	Sophus::SE3d::Point trans = _tform.translation();
	QuaternionType quat = GetQuaternion();
	FixedVectorType<VectorDimension> v;
	v << trans(0), trans(1), trans(2), quat.w(), quat.x(), quat.y(), quat.z();
	return v;
}

PoseSE3::Transform PoseSE3::ToTransform() const 
{
	return Transform( _tform.matrix() );
}

PoseSE3 PoseSE3::Inverse() const 
{
	PoseSE3 ret;
	ret._tform = _tform.inverse();
	return ret;
}

PoseSE3 PoseSE3::Exp( const PoseSE3::TangentVector& tangent ) 
{
	PoseSE3 out;
	out._tform = Sophus::SE3d::exp( tangent );
	return out;
}

PoseSE3::TangentVector PoseSE3::Log( const PoseSE3& pose ) 
{
	return Sophus::SE3d::log( pose._tform );
}

PoseSE3::AdjointMatrix PoseSE3::Adjoint( const PoseSE3& other )
{
	return other._tform.Adj();
}

PoseSE3 PoseSE3::operator*( const PoseSE3& other ) const 
{
	PoseSE3 out;
	out._tform = _tform * other._tform;
	return out;
}

std::ostream& operator<<(std::ostream& os, const PoseSE3& se3) 
{
	os << se3.ToVector().transpose();
	return os;
}
	
} // end namespace argus

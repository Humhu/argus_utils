#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"
#include <cmath>

namespace argus
{

PoseSE2::PoseSE2() 
{
	FromTerms( 0, 0, 0 );
}

PoseSE2::PoseSE2(const double x, const double y, const double theta) 
{
	FromTerms( x, y, theta );
}

PoseSE2::PoseSE2(const VectorType &vec) 
{
	if( vec.size() != 3 )
	{
		throw std::runtime_error( "PoseSE2 must be constructed from 3-element vector." );
	}
	FromTerms( vec(0), vec(1), vec(2) );
}

PoseSE2::PoseSE2( const MatrixType& m ) 
{
	if( m.cols() != 3 || m.rows() != 3 )
	{
		throw std::runtime_error( "PoseSE2 must be constructed from 3x3 matrix.");
	}
	// This is not implemented in Eigen for some reason
	//_tform = Transform( m );
	Translation2Type trans( m(0,2), m(1,2) );
	Rotation rot( 0 );
	rot.fromRotationMatrix( m.topLeftCorner<2,2>() );
	_tform = trans * rot;
}
	
// TODO Remove this because of inaccuracy!
PoseSE2::PoseSE2( const Transform& t ) 
: _tform( t )
{}

PoseSE2::PoseSE2( const Translation2Type& t, const Rotation& r ) 
: _tform( t * r )
{}

PoseSE2 PoseSE2::FromSE3(const PoseSE3& se3) 
{
	// Approximate projection by taking angular norm * z component
	Translation3Type trans3 = se3.GetTranslation();
	Translation2Type trans2( trans3.x(), trans3.y() );
	Rotation rot( 0 );

	QuaternionType quat = se3.GetQuaternion();
	double rotmag_2 = acos(quat.w());
	//double vnorm = quat.vec().norm();
	double vnorm = sin( rotmag_2 );
	if( vnorm > 1E-9 )
	{
		rot = Rotation( 2 * rotmag_2 * quat.z() / vnorm );
	}
	return PoseSE2( trans2, rot );
}

PoseSE2::TangentVector PoseSE2::Log( const PoseSE2& pose ) 
{
	FixedVectorType<3> vec = pose.ToVector();
	double x = vec(0);
	double y = vec(1);
	double theta = vec(2);

	SECoefficients coeffs( theta );
	double f;
	if( theta*theta < 1E-4 ) 
	{
		f = 1 - theta*theta/12 - theta*theta/720;
	} 
	else 
	{
		f = 0.5*coeffs.a/coeffs.b;
	}
	
	TangentVector v;
	v << f*x + 0.5*theta*y,
	     f*y - 0.5*theta*x,
	     theta;
	return v;
}

PoseSE2 PoseSE2::Exp( const PoseSE2::TangentVector& tangent ) 
{
	double theta = tangent(2);
	SECoefficients coeffs( theta );
	FixedMatrixType<2,2> V;
	V << coeffs.a, -coeffs.b*theta,
	     coeffs.b*theta, coeffs.a;
	FixedVectorType<2> t = V*tangent.block<2,1>(0,0);

	return PoseSE2( t(0), t(1), tangent(2) );
}

PoseSE2::AdjointMatrix PoseSE2::Adjoint( const PoseSE2& pose ) 
{
	AdjointMatrix m = AdjointMatrix::Zero();
	m.block<2,2>(0,0) = pose._tform.rotation();
	m(0,2) = pose._tform.translation().y();
	m(1,2) = -pose._tform.translation().x();
	m(2,2) = 1;
	return m;
}

Translation2Type PoseSE2::GetTranslation() const 
{
	return Translation2Type( _tform.translation() );
}

PoseSE2::Rotation PoseSE2::GetRotation() const 
{
	Rotation rot( 0 );
	rot.fromRotationMatrix( _tform.rotation() );
	return rot;
}

const PoseSE2::Transform& PoseSE2::ToTransform() const 
{
	return _tform;
}

FixedVectorType<PoseSE2::VectorDimension> 
PoseSE2::ToVector() const 
{
	Transform::ConstTranslationPart trans = _tform.translation();
	Rotation rot( 0 );
	rot.fromRotationMatrix( _tform.rotation() );

	FixedVectorType<PoseSE2::VectorDimension> ret;
	ret << trans.x(), trans.y(), rot.angle();
	return ret;
}

PoseSE2 PoseSE2::Inverse() const 
{
	PoseSE2 ret( _tform.inverse() );
	return ret;
}

PoseSE2 PoseSE2::operator+() const 
{
	return *this;
}

PoseSE2 PoseSE2::operator-() const 
{
	return Inverse();
}

PoseSE2 PoseSE2::operator*(const PoseSE2& other) const 
{
	return PoseSE2( _tform * other._tform );
}

PoseSE2 PoseSE2::operator/(const PoseSE2& other) const 
{
	return PoseSE2( _tform * other._tform.inverse() );
}

std::ostream& operator<<(std::ostream& os, const PoseSE2& se2) 
{
	VectorType vec = se2.ToVector();
	os << vec(0) << " " << vec(1) << " " << vec(2);
	return os;
}

void PoseSE2::FromTerms( double x, double y, double yaw )
{
	Translation2Type trans( x, y );
	Rotation rot( yaw );
	_tform = trans * rot;
}

}

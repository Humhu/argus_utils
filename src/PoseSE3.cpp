#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include <cmath>

namespace argus 
{

PoseSE3::PoseSE3()
: PoseSE3( 0, 0, 0, 1, 0, 0, 0 ) {}
	
PoseSE3::PoseSE3( double x, double y, double z, double qw, 
                  double qx, double qy, double qz )
{
	FromTerms( x, y, z, qw, qx, qy, qz );
}

PoseSE3::PoseSE3( const VectorType& vec ) 
{
	FromTerms( vec(0), vec(1), vec(2), vec(3), vec(4),
	           vec(5), vec(6) );
}

void PoseSE3::FromTerms( double x, double y, double z, double qw,
                         double qx, double qy, double qz )
{
	QuaternionType quat( qw, qx, qy, qz );
	quat.normalize();
	Translation3Type trans( x, y, z );
	Translation3Type zero( 0, 0, 0 );
	_tform = trans*(quat*zero);
}

PoseSE3::PoseSE3( const Transform& t ) 
: _tform( t ) {}

PoseSE3::PoseSE3( const MatrixType& mat ) 
{
	if( mat.rows() == 3 && mat.cols() == 3 )
	{
		FromMat3( FixedMatrixType<3,3>( mat ) );
	}
	if( mat.rows() == 4 || mat.cols() == 4 )
	{
		FromMat4( FixedMatrixType<4,4>( mat ) );
	}
	else
	{
		throw std::runtime_error( "PoseSE3: Invalid matrix input." );
	}
}

PoseSE3::PoseSE3( const FixedMatrixType<4,4>& mat ) 
{
	FromMat4( mat );
}

PoseSE3::PoseSE3( const FixedMatrixType<3,3>& rot )
{
	FromMat3( rot );
}

void PoseSE3::FromMat4( const FixedMatrixType<4,4>& mat )
{
	_tform = Transform( mat );
}

void PoseSE3::FromMat3( const FixedMatrixType<3,3>& rot )
{
	_tform = Transform( Translation3Type( 0, 0, 0 ) * QuaternionType( rot ) );
}

PoseSE3::PoseSE3( const Translation3Type& t, const QuaternionType& q ) 
{
	Translation3Type trans( t );
	QuaternionType quat( q );
	quat.normalize();
	Translation3Type zero(0,0,0);
	Transform qTrans = quat*zero;
	_tform = trans*qTrans;
}

PoseSE3 PoseSE3::FromSE2( const PoseSE2& se2 ) 
{
	FixedVectorType<3> v = se2.ToVector();
	QuaternionType quat( Eigen::AngleAxisd( v(2), Eigen::Vector3d::UnitZ()) );
	Translation3Type trans( v(0), v(1), 0.0);
	return PoseSE3( trans, quat );
}

Translation3Type PoseSE3::GetTranslation() const 
{
	return Translation3Type( _tform.translation() );
}

QuaternionType PoseSE3::GetQuaternion() const 
{
	QuaternionType quat( _tform.linear() );
	quat.normalize();
	return quat;
}

FixedVectorType<7> PoseSE3::ToVector() const 
{
	FixedVectorType<7> v;
	Transform::ConstTranslationPart trans = _tform.translation();
	QuaternionType quat = GetQuaternion();
	v << trans(0), trans(1), trans(2), quat.w(), quat.x(), quat.y(), quat.z();
	return v;
}

const PoseSE3::Transform& PoseSE3::ToTransform() const 
{
	return _tform;
}

PoseSE3 PoseSE3::Inverse() const 
{
	PoseSE3 ret( _tform.inverse() );
	return ret;
}

PoseSE3 PoseSE3::Exp( const PoseSE3::TangentVector& tangent ) 
{
	FixedVectorType<3> transVec = tangent.block<3,1>(0,0);
	FixedVectorType<3> rotVec = tangent.block<3,1>(3,0);
	
	double theta = rotVec.norm();
	SECoefficients coeffs( theta );
	
	QuaternionType::Matrix3 wx = cross_product_matrix(rotVec);
	QuaternionType::Matrix3 R = QuaternionType::Matrix3::Identity() + coeffs.a*wx + coeffs.b*wx*wx;
	QuaternionType::Matrix3 V = QuaternionType::Matrix3::Identity() + coeffs.b*wx + coeffs.c*wx*wx;
	
	// MatrixType m = MatrixType::Identity( 6, 6 );
	// m.block<3,3>(0,0) = R;
	// m.block<3,1>(0,3) = V*u;
	// PoseSE3 ret(m);

	QuaternionType rot( R );
	Translation3Type trans( V*transVec );
	return PoseSE3( trans, rot );
}

PoseSE3::TangentVector PoseSE3::Log( const PoseSE3& pose ) 
{
	PoseSE3::Transform::LinearMatrixType R = pose.ToTransform().rotation();
	
	FixedVectorType<3> w;
	w << 0.5 * (R(2,1) - R(1,2)),
	0.5 * (R(0,2) - R(2,0)),
	0.5 * (R(1,0) - R(0,1));
	
	double ct = 0.5  * (R.trace() - 1);
	double st = w.norm();
	double st2 = st*st;
	
	// From code by Ethan Eade (?)
	if (ct > 0.999856) // TODO Make #def
	{
		// Small angles
		// Taylor expansion of f(x) = arcsin(x) / x
		// x^2 = st2
		double f = 1 + st2*( 1/6.0 + st2*(3/40.0 + st2*5/112.0) );
		w *= f;
	} 
	else if (ct > -0.99) 
	{
		w *= acos(ct)/sqrt(st);
	} 
	else 
	{
		// Angles near pi
		double theta = M_PI - asin(st);
		double invB = (theta*theta) / (1 - ct);
		
		double w00 = invB*(R(0,0) - ct);
		double w11 = invB*(R(1,1) - ct);
		double w22 = invB*(R(2,2) - ct);
		
		double w01 = invB*0.5*(R(0,1) + R(1,0));
		double w02 = invB*0.5*(R(0,2) + R(2,0));
		double w12 = invB*0.5*(R(1,2) + R(2,1));
		
		// Take sqrt of biggest element of w
		if (w00 > w11 && w00 > w22) 
		{
			w(0) = (w(0) < 0 ? -1 : 1) * sqrt(w00);
			w(1) = w01/w(0);
			w(2) = w02/w(0);
		} 
		else if (w11 > w22 && w11 > w00) 
		{
			w(1) = (w(1) < 0 ? -1 : 1) * sqrt(w11);
			w(0) = w01/w(1);
			w(2) = w12/w(1);
		} 
		else 
		{
			w(2) = (w(2) < 0 ? -1 : 1) * sqrt(w22);
			w(0) = w02 / w(2);
			w(1) = w12 / w(2);
		}
	}
	
	double theta = w.norm();
	SECoefficients coeffs( theta );
	
	QuaternionType::Matrix3 wx = cross_product_matrix( w );
	QuaternionType::Matrix3 V = QuaternionType::Matrix3::Identity() + coeffs.b*wx + coeffs.c*wx*wx;
	FixedVectorType<3> t = pose.GetTranslation().vector();
	FixedVectorType<3> u = V.partialPivLu().solve(t);
	
	PoseSE3::TangentVector v;
	v.block<3,1>(0,0) = u;
	v.block<3,1>(3,0) = w;
	return v;
}

PoseSE3::AdjointMatrix PoseSE3::Adjoint( const PoseSE3& other )
{
	AdjointMatrix m = AdjointMatrix::Zero();
	m.block<3,3>(0,0) = other._tform.linear();
	m.block<3,3>(3,3) = other._tform.linear();
	FixedVectorType<3> t = other._tform.translation();
	m.block<3,3>(0,3) = cross_product_matrix( t ) * other._tform.linear();
	return m;
}

PoseSE3 PoseSE3::operator+() const 
{
	return *this;
}

PoseSE3 PoseSE3::operator-() const 
{
	return Inverse();
}

PoseSE3 PoseSE3::operator*( const PoseSE3& other ) const 
{
	Transform tTrans = ToTransform();
	Transform oTrans = other.ToTransform();
	return PoseSE3( tTrans*oTrans );
}

PoseSE3 PoseSE3::operator/( const PoseSE3& other ) const 
{
	Transform tTrans = ToTransform();
	Transform oTrans = other.ToTransform().inverse(Eigen::Isometry);
	return PoseSE3( tTrans*oTrans );
}

std::ostream& operator<<(std::ostream& os, const PoseSE3& se3) 
{
	os << se3.ToVector().transpose();
	return os;
}

template <class C>
Eigen::Matrix<C,3,3> cross_product_matrix( const Eigen::Matrix<C,3,1>& v ) 
{
	Eigen::Matrix<C,3,3> s;
	s << 0, -v(2), v(1),
			v(2), 0, -v(0),
			-v(1), v(0), 0;
	return s;
}

// From code by Ethan Eade
SECoefficients::SECoefficients( double theta ) 
{
	double tt = theta*theta;
	
	if( tt < 1E-6 ) 
	{
		b = 0.5 + tt/24 - tt*tt/720;
		c = 1.0/6 - tt/120 + tt*tt/5040;
		a = 1 - tt*c;
	} 
	else 
	{
		a = sin(theta)/theta;
		b = (1 - cos(theta))/tt;
		c = (1 - a)/tt;
	}
}
	
} // end namespace argus

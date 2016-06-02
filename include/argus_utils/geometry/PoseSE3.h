#pragma once

#include <Eigen/Geometry>
#include <iostream>

#include "argus_utils/geometry/GeometryTypes.h"
#include "argus_utils/utils/LinalgTypes.h"

// @todo: Deprecate Pose and move to sophus::se3d
// #include <sophus/se3.hpp>

namespace argus
{

class PoseSE2;

/*! \brief Represents a 6D pose. Provides various methods to calculate
 * geometric properties, such as tangent velocities. */
class PoseSE3 
{
friend class PoseSE2;
public:

	static const int VectorDimension = 7;
	static const int TangentDimension = 6;
	
	// Representation is [x, y, z, qw, qx, qy, qz]
	typedef double ScalarType; // TODO Templatize
	
	typedef Eigen::Transform<ScalarType, 3, Eigen::Isometry> Transform;

	// Probability definitions
	typedef FixedMatrixType<TangentDimension, TangentDimension> CovarianceMatrix;
	
	// Lie group definitions
	typedef FixedVectorType<TangentDimension> TangentVector;
	typedef FixedMatrixType<TangentDimension, TangentDimension> AdjointMatrix;
	
	/*! \brief Creates an SE3 object with 0 translation and rotation. */
	PoseSE3();

	/*! \brief Creates an SE3 object with the specified values. */
	PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz );

	/*! \brief Creates an SE3 object from an Eigen transform. */
	explicit PoseSE3( const Transform& trans );

	/*! \brief Creates an SE3 object from a matrix assumed to be a 4x4 homogeneous matrix
	 * or a 3x3 rotation matrix. */
	explicit PoseSE3( const MatrixType& mat );

	/*! \brief Creates an SE3 object from a 3x3 rotation matrix with
	 * zero translation. */
	explicit PoseSE3( const FixedMatrixType<3,3>& rot );

	/*! \brief Creates an SE3 object from a 4x4 homogeneous matrix. */
	explicit PoseSE3( const FixedMatrixType<4,4>& mat );

	/*! \brief Creates an SE3 object from a translation and quaternion object. */
	explicit PoseSE3( const Translation3Type& t, const QuaternionType& q );
	explicit PoseSE3( const PoseSE2& se2 );

	Transform ToTransform() const;
	FixedVectorType<VectorDimension> ToVector() const; //[x,y,z,qw,qx,qy,qz]
	PoseSE3 Inverse() const;

	Translation3Type GetTranslation() const;
	QuaternionType GetQuaternion() const;

	/*! \brief Exponential a tangent vector to an SE3 object. */
	static PoseSE3 Exp( const TangentVector& other );

	/*! \brief Return the logarithm of an SE3 object. */
	static TangentVector Log( const PoseSE3& other );
	
	/*! \brief Return the adjoint matrix of an SE3 object. */
	static AdjointMatrix Adjoint( const PoseSE3& other );

	PoseSE3 operator+() const;
	PoseSE3 operator-() const;
	PoseSE3 operator*( const PoseSE3& other ) const;
	PoseSE3 operator/( const PoseSE3& other ) const;
	
protected:

	Transform _tform;

	/*! \brief Initialize this SE3 object from a 3x3 rotation matrix. */
	void FromMat3( const FixedMatrixType<3,3>& rot );
	/*! \brief Initialize this SE3 object from a 4x4 homogeneous matrix. */
	void FromMat4( const FixedMatrixType<4,4>& mat );

};

std::ostream& operator<<( std::ostream& os, const PoseSE3& se3 );

template<class C>
Eigen::Matrix<C,3,3> cross_product_matrix( const Eigen::Matrix<C,3,1>& v );

// Helper for exp functions
struct SECoefficients {
	double a;
	double b;
	double c;
	SECoefficients( double theta );
};
	
}

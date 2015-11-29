#pragma once

#include <Eigen/Geometry>
#include <iostream>

// TODO Deprecate Pose and move to sophus::se3d
// #include <sophus/se3.hpp>

namespace argus_utils
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
		static const int CovarianceDimension = 21;
		
		// Representation is [x, y, z, qw, qx, qy, qz]
		typedef double ScalarType; // TODO Templatize
		typedef Eigen::Matrix<ScalarType, VectorDimension, 1> Vector;
		typedef Eigen::Matrix<ScalarType, 3, 1> TranslationVector;
		typedef Eigen::Matrix<ScalarType, 3, 1> AxisVector;
		
		typedef Eigen::Transform<ScalarType, 3, Eigen::Isometry> Transform;
		typedef Eigen::Matrix<ScalarType, 4, 4> Matrix;
		typedef Eigen::Quaternion<ScalarType> Quaternion;
		typedef Eigen::Translation<ScalarType, 3> Translation;

		// Probability definitions
		typedef Eigen::Matrix<ScalarType, TangentDimension, TangentDimension> CovarianceMatrix;
		typedef Eigen::Matrix<ScalarType, CovarianceDimension, 1> CovarianceVector;
		
		// Lie group definitions
		typedef Eigen::Matrix<ScalarType, TangentDimension, 1> TangentVector;
		typedef Eigen::Matrix<ScalarType, TangentDimension, TangentDimension> AdjointMatrix;
		
		PoseSE3();
		PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz );
		explicit PoseSE3( const Transform& trans );
		explicit PoseSE3( const Matrix& mat );
		explicit PoseSE3( const Translation& t, const Quaternion& q );
		explicit PoseSE3( const PoseSE2& se2 );

		Transform ToTransform() const;
		Vector ToVector() const; //[x,y,z,qw,qx,qy,qz]
		PoseSE3 Inverse() const;

		PoseSE3::Translation GetTranslation() const;
		PoseSE3::Quaternion GetQuaternion() const;

		static PoseSE3 Exp( const TangentVector& other );
		static PoseSE3::TangentVector Log( const PoseSE3& other );
		static PoseSE3::AdjointMatrix Adjoint( const PoseSE3& other );

		PoseSE3 operator+() const;
		PoseSE3 operator-() const;
		PoseSE3 operator*( const PoseSE3& other ) const;
		PoseSE3 operator/( const PoseSE3& other ) const;
		
	protected:

		Transform tform;

	};

	std::ostream& operator<<( std::ostream& os, const PoseSE3& se3 );
	
	// TODO Implement!
	PoseSE3::CovarianceVector unroll_covariance( const PoseSE3::CovarianceMatrix& cov );
	PoseSE3::CovarianceMatrix rollup_covariacne( const PoseSE3::CovarianceVector& vec );

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

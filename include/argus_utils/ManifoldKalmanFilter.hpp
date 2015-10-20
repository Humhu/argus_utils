#pragma once

#include <Eigen/Dense>
#include <boost/function.hpp>

namespace argus_utils
{

/*! \brief Specifies whether noise is composed before or after the mean. */
enum ManifoldNoiseFrame
{
	BodyFrame = 0,
	WorldFrame,
	//BothFrames // TODO Allow tracking of body and world uncertainties separately
};

template < class ManifoldType >
typename ManifoldType::CovarianceMatrix
ConvertBodyToWorld( const ManifoldType& pose, 
                    const typename ManifoldType::CovarianceMatrix& cov )
{
	typename ManifoldType::AdjointMatrix adj = pose.GetAdjoint();
	return adj * cov * adj.transpose();
}

template < class ManifoldType >
typename ManifoldType::CovarianceMatrix
ConvertWorldToBody( const ManifoldType& pose, 
                    const typename ManifoldType::CovarianceMatrix& cov )
{
	typename ManifoldType::AdjointMatrix invAdj = pose.GetAdjoint().inverse();
	return invAdj * cov * invAdj.transpose();
}

// TODO Use this object for means and observations?
// TODO Templatize noiseFrame to avoid branches
/*! \brief Represents a group object with tangent noise. Noise can be either
 * pre-multiplied (body) or post-multiplied (world). */
template < class ManifoldType >
class ManifoldNoise
{
public:

	typedef typename ManifoldType::CovarianceMatrix CovarianceMatrix;
	
	const ManifoldType mean;
	const CovarianceMatrix covariance;
	const ManifoldNoiseFrame noiseFrame;
	
	ManifoldNoise( const ManifoldType& m, const CovarianceMatrix& cov,
	               ManifoldNoiseFrame n )
	: mean( m ), covariance( cov ), noiseFrame( n ) {}
	
	ManifoldNoise ConvertToBody() const
	{
		if( noiseFrame == BodyFrame ) { return ManifoldNoise( *this ); }
		else
		{
			CovarianceMatrix converted = ConvertWorldToBody<ManifoldType>( mean, covariance );
			return ManifoldNoise( mean, converted, BodyFrame );
		}
	}
	
	ManifoldNoise ConvertToWorld() const
	{
		if( noiseFrame == WorldFrame ) { return ManifoldNoise( *this ); }
		else
		{
			CovarianceMatrix converted = ConvertBodyToWorld<ManifoldType>( mean, covariance );
			return ManifoldNoise( mean, converted, WorldFrame );
		}
	}
};

// TODO Clean up interface and defaults
// TODO Consolidate into single filter without specialization using ManifoldNoise
/*! \class ManifoldKalmanFilter ManifoldKalmanFilter.h
* \brief A basic discrete-time Kalman filter with compile-time sizes. */
template < class ManifoldType, int StateNoiseFrame = WorldFrame >
class ManifoldKalmanFilter {};

template < class ManifoldType >
class ManifoldKalmanFilter< ManifoldType, BodyFrame >
{
public:
	
	typedef typename ManifoldType::ScalarType Scalar;
	typedef typename ManifoldType::CovarianceMatrix StateCovariance;
	typedef Eigen::Matrix<Scalar, ManifoldType::TangentDimension, 
	                      ManifoldType::TangentDimension>   
	        ObservationCovariance;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	ManifoldKalmanFilter() {}

	StateCovariance& TransCovariance() { return Q; }
	ObservationCovariance& ObsCovariance() { return R; }
	
	ManifoldType& EstimateMean() { return x; }
	StateCovariance& EstimateCovariance() { return S; }

	void Predict()
	{
		Predict( Q, BodyFrame );
	}
	
	/*! \brief Specify the frame which noise should be added. */
	void Predict( const StateCovariance& q, ManifoldNoiseFrame frame )
	{
		if( frame == BodyFrame )
		{
			S += q;
		}
		else if( frame == WorldFrame )
		{
			S += ConvertWorldToBody<ManifoldType>( x, q );
		}
		else
		{
			throw std::runtime_error( "Invalid frame for predict" );
		}
	}
	
	/*! \brief Perform a displacement in the body frame (post multiply) */
	void PredictBody( const ManifoldType& d )
	{
		PredictBody( d, Q, BodyFrame );
	}
	
	void PredictBody( const ManifoldType& d, const StateCovariance& q, ManifoldNoiseFrame frame )
	{
		x = x * d;
		Predict( q, frame );
	}
	
	void PredictWorld( const ManifoldType& d )
	{
		PredictWorld( d, Q, BodyFrame );
	}
	
	void PredictWorld( const ManifoldType& d, const StateCovariance& q, ManifoldNoiseFrame frame )
	{
		x = d * x;
		Predict( q, frame );
	}
	
	void UpdateBody( const ManifoldType& z, ManifoldNoiseFrame frame = BodyFrame )
	{
		UpdateBody( z, R, frame );
	}
	
	/*! \brief Performs an update in the body frame. */
	void UpdateBody( const ManifoldType& z, const ObservationCovariance& r,
	                 ManifoldNoiseFrame frame = BodyFrame )
	{
		ManifoldType err = x.Inverse()*z;
		typename ManifoldType::TangentVector e = ManifoldType::Log( err );
		
		StateCovariance K;
		if( frame == BodyFrame )
		{
			// All noise is in body frame, good to go
			K = S*( (S + r).inverse() );
		}
		else if( frame == WorldFrame )
		{
			// Need to convert obs noise to body frame
			K = S*( (S + ConvertWorldToBody<ManifoldType>( z, r )).inverse() );
		}
		else
		{
			throw std::runtime_error( "Invalid frame for update body" );
		}
		x = x * ManifoldType::Exp( K*e );
		S = ( StateCovariance::Identity() - K ) * S;
		
	}
	
	void UpdateWorld( const ManifoldType& z, ManifoldNoiseFrame frame = BodyFrame )
	{
		UpdateWorld( z, R, frame );
	}
	
	/*! \brief Performs an update in the world frame. */
	void UpdateWorld( const ManifoldType& z, const ObservationCovariance& r,
	                  ManifoldNoiseFrame frame = BodyFrame )
	{
		ManifoldType err = z*x.Inverse();
		typename ManifoldType::TangentVector e = ManifoldType::Log( err );
		
		StateCovariance K;
		StateCovariance Sworld = ConvertBodyToWorld( x, S );
		if( frame == BodyFrame )
		{
			// Need to convert both noises to world frame
			K = Sworld*( (Sworld + ConvertBodyToWorld( z, r )).inverse() );
		}
		else if( frame == WorldFrame )
		{
			// Need to convert estimate noise to world frame
			K = Sworld*( (Sworld + r).inverse() );
		}
		else
		{
			throw std::runtime_error( "Invalid frame for update world" );
		}
		x = ManifoldType::Exp( K*e ) * x;
		S = ( StateCovariance::Identity() - K ) * S;
	}
	
private:

	/*! \brief The current state estimate mean and covariance. */
	ManifoldType x;
	StateCovariance S;

	/*! \brief The state transition covariance matrix. */
	StateCovariance Q;

	/*! \brief The observation covariance matrix. */
	ObservationCovariance R;

};

template < class ManifoldType >
class ManifoldKalmanFilter< ManifoldType, WorldFrame >
{
public:
	
	typedef typename ManifoldType::ScalarType Scalar;
	typedef Eigen::Matrix<Scalar, ManifoldType::TangentDimension, 
	                      ManifoldType::TangentDimension>   
	        StateCovariance;
	typedef Eigen::Matrix<Scalar, ManifoldType::TangentDimension, 
	                      ManifoldType::TangentDimension>   
	        ObservationCovariance;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	ManifoldKalmanFilter() {}

	StateCovariance& TransCovariance() { return Q; }
	ObservationCovariance& ObsCovariance() { return R; }
	
	ManifoldType& EstimateMean() { return x; }
	StateCovariance& EstimateCovariance() { return S; }

	void Predict()
	{
		Predict( Q, WorldFrame );
	}
	
	/*! \brief Specify the frame which noise should be added. */
	void Predict( const StateCovariance& q, ManifoldNoiseFrame frame )
	{
		if( frame == WorldFrame )
		{
			S += q;
		}
		else if( frame == BodyFrame )
		{
			// Need to convert noise into world frame
			S += ConvertBodyToWorld<ManifoldType>( x, q );
		}
		else
		{
			throw std::runtime_error( "Invalid frame." );
		}
	}
	
	/*! \brief Perform a displacement in the body frame (post multiply) */
	void PredictBody( const ManifoldType& d )
	{
		PredictBody( d, Q, BodyFrame );
	}
	
	void PredictBody( const ManifoldType& d, const StateCovariance& q, ManifoldNoiseFrame frame )
	{
		x = x * d;
		Predict( q, frame );
	}
	
	void PredictWorld( const ManifoldType& d )
	{
		PredictWorld( d, Q, BodyFrame );
	}
	
	void PredictWorld( const ManifoldType& d, const StateCovariance& q, ManifoldNoiseFrame frame )
	{
		x = d * x;
		Predict( q, frame );
	}
	
	void UpdateBody( const ManifoldType& z, ManifoldNoiseFrame frame = WorldFrame )
	{
		UpdateBody( z, R, frame );
	}
	
	/*! \brief Performs an update in the body frame. */
	void UpdateBody( const ManifoldType& z, const ObservationCovariance& r,
	                 ManifoldNoiseFrame frame )
	{
		ManifoldType err = x.Inverse()*z;
		typename ManifoldType::TangentVector e = ManifoldType::Log( err );
		
		StateCovariance K;
		StateCovariance Sbody = ConvertWorldToBody<ManifoldType>( x, S );
		if( frame == BodyFrame )
		{
			// Need to convert estimate noise to body frame
			K = Sbody*( (Sbody + r).inverse() );
		}
		else if( frame == WorldFrame )
		{
			// Need to convert both noises to body frame
			K = Sbody*( (Sbody + ConvertWorldToBody<ManifoldType>( z, r )).inverse() );
		}
		else
		{
			throw std::runtime_error( "Invalid frame for update body" );
		}
		x = x * ManifoldType::Exp( K*e );
		S = ( StateCovariance::Identity() - K ) * S;
	}
	
	void UpdateWorld( const ManifoldType& z, ManifoldNoiseFrame frame )
	{
		UpdateWorld( z, R, frame );
	}
	
	/*! \brief Performs an update in the world frame. */
	void UpdateWorld( const ManifoldType& z, const ObservationCovariance& r,
	                  ManifoldNoiseFrame frame )
	{
		ManifoldType err = z*x.Inverse();
		typename ManifoldType::TangentVector e = ManifoldType::Log( err );
		
		StateCovariance K;
		if( frame == BodyFrame )
		{
			// Need to convert observation noise to world frame
			K = S*( (S + ConvertBodyToWorld( z, r )).inverse() );
		}
		else if( frame == WorldFrame )
		{
			// All noise in world frame, good to go
			K = S*( (S + r).inverse() );
		}
		else
		{
			throw std::runtime_error( "Invalid frame for update world" );
		}
		x = ManifoldType::Exp( K*e ) * x;
		S = ( StateCovariance::Identity() - K ) * S;
	}
	
private:

	/*! \brief The current state estimate mean and covariance. */
	ManifoldType x;
	StateCovariance S;

	/*! \brief The state transition covariance matrix. */
	StateCovariance Q;

	/*! \brief The observation covariance matrix. */
	ObservationCovariance R;

};

}

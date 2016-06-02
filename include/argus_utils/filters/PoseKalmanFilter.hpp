#pragma once

#include "argus_utils/geometry/PoseNoise.hpp"
#include <Eigen/Cholesky>

namespace argus
{

template <typename Pose>
class PoseKalmanFilter
{
public:

	typedef Pose PoseType;
	typedef PoseNoise<PoseType> PoseNoiseType;
	typedef typename PoseNoiseType::CovarianceType CovarianceType;

	PoseKalmanFilter( const PoseType& pose, const CovarianceType& cov, 
	                  PoseNoiseFrame frame ) 
	: _estimate( pose, cov, frame ) {}

	PoseKalmanFilter( const PoseNoiseType& est )
	: _estimate( est ) {}

	PoseNoiseType& Estimate() { return _estimate; }
	const PoseNoiseType& Estimate() const { return _estimate; }

	/*! \brief Perform a predict step where a body (right) displacement
	 * is applied to the current estimate. */
	void PredictBodyDisplace( const PoseNoiseType& displacement )
	{
		_estimate = PoseNoiseType::Compose( _estimate, displacement, _noiseFrame );
	}

	/*! \brief Perform a predict step where a world (left) displacement
	 * is applied to the current estimate. */
	void PredictWorldDisplace( const PoseNoiseType& displacement )
	{
		_estimate = PoseNoiseType::Compose( displacement, _estimate, _noiseFrame );
	}

	void Update( const PoseNoiseType& obs )
	{
		if( obs.Frame() == BodyNoiseFrame )
		{
			UpdateBody( obs.Pose(), obs.Covariance() );
		}
		else
		{
			UpdateWorld( obs.Pose(), obs.Covariance() );
		}
	}

	/*! \brief Perform an update using an observation with noise in
	 * the body frame. */
	void UpdateBody( const PoseType& obs, const CovarianceType& R )
	{
		// Error computed in body frame
		PoseNoiseType est = _estimate.ToBody();
		PoseType x = est.Pose();
		CovarianceType S = est.Covariance();

		PoseType err = x.Inverse() * obs;
		typename PoseType::TangentVector v = PoseType::Log( err );

		CovarianceType V = S + R;
		Eigen::LDLT<CovarianceType> Vinv( V );
		
		//CovarianceType K = S * V.inverse();
		//PoseType xup = x * PoseType::Exp( K*v );
		//CovarianceType Sup = ( CovarianceType::Identity() - K ) * S;

		// More stable form of update equations
		// Since error is in body frame, we right-multiply correction
		PoseType xup = x * PoseType::Exp( S * Vinv.solve( v ) );
		CovarianceType Sup = S - S * Vinv.solve( S );

		PoseNoiseType updated( xup, Sup, BodyNoiseFrame );
		_estimate = updated.ToFrame( _noiseFrame );
	}

	/*! \brief Perform an update using an observation with noise in
	 * the world frame. */
	void UpdateWorld( const PoseType& obs, const CovarianceType& R )
	{
		// Error computed in world frame
		PoseNoiseType est = _estimate.ToWorld();
		PoseType x = est.Pose();
		CovarianceType S = est.Covariance();

		PoseType err = obs * x.Inverse();
		typename PoseType::TangentVector v = PoseType::Log( err );

		CovarianceType V = S + R;
		Eigen::LDLT<CovarianceType> Vinv( V );
		
		//CovarianceType K = S * V.inverse();
		//PoseType xup = PoseType::Exp( K*v ) * x;
		//CovarianceType Sup = ( CovarianceType::Identity() - K ) * S;

		// More stable form of update equations
		// Since error is in world frame, we left-multiply correction
		PoseType xup = PoseType::Exp( S * Vinv.solve( v ) ) * x;
		CovarianceType Sup = S - S * Vinv.solve( S );

		PoseNoiseType updated( xup, Sup, WorldNoiseFrame );
		_estimate = updated.ToFrame( _noiseFrame );
	}

private:

	PoseNoiseType _estimate; // Stores both the mean and covariance
	PoseNoiseFrame _noiseFrame; // The frame to track noise in

};

}

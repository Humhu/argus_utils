#pragma once

#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

/*! \brief Specifies whether noise is composed before or after the mean. */
enum PoseNoiseFrame
{
	BodyNoiseFrame, // Post-multiply
	WorldNoiseFrame, // Pre-multiply
	//BothFrames // TODO Allow tracking of body and world uncertainties separately
};

/*!
 * \brief Represents a pose element with noise as the pose element mean
 * pre (right) or post (left) multiplied by an exponentiated random vector. 
 * In our convention, pre-multiplied corresponds to noise in the world frame 
 * and post-multiplied corresponds to noise in the body frame. 
 */
template <typename P>
class PoseNoise
{
public:

	typedef P PoseType;
	static const int NoiseDim = PoseType::TangentDimension;
	typedef FixedMatrixType<NoiseDim, NoiseDim> CovarianceType;

	/*! \brief Create a new PoseNoise object with the specified pose,
	 * covariance, and frame. */
	PoseNoise( const PoseType& pose, const CovarianceType& cov,
	           PoseNoiseFrame frame )
	: _pose( pose ), _cov ( cov ), _frame ( frame ) {}

	const PoseType& Pose() const { return _pose; }
	const CovarianceType& Covariance() const { return _cov; }
	const PoseNoiseFrame Frame() const { return _frame; }

	bool IsBody() const { return _frame == BodyNoiseFrame; }
	bool IsWorld() const { return _frame == WorldNoiseFrame; }

	/*! \brief Return a copy of this object converted to the specified frame. */
	PoseNoise ToFrame( PoseNoiseFrame frame ) const
	{
		if( frame == BodyNoiseFrame ) { return ToBody(); }
		if( frame == WorldNoiseFrame ) { return ToWorld(); }
		else
		{
			throw std::runtime_error( "PoseNoise: Invalid frame." );
		}
	}

	PoseNoise ToBody() const
	{
		if( _frame == BodyNoiseFrame ) { return *this; }
		// Else move noise from left to right with inverse adjoint
		else
		{
			return PoseNoise( _pose, WorldCovToBody( _cov, _pose ), 
			                  BodyNoiseFrame );
		}
	}

	PoseNoise ToWorld() const
	{
		if( _frame == WorldNoiseFrame ) { return *this; }
		// Else move noise from right to left with adjoint
		else
		{
			return PoseNoise( _pose, BodyCovToWorld( _cov, _pose ), 
			                  WorldNoiseFrame );
		}
	}

	/*! \brief Return a new object equal to the two poses composed with noise
	 * in the specified final frame. */
	static PoseNoise Compose( const PoseNoise& lhs, const PoseNoise& rhs,
	                   PoseNoiseFrame frame )
	{
		if( frame == BodyNoiseFrame ) { return ComposeToBody( lhs, rhs ); }
		if( frame == WorldNoiseFrame ) { return ComposeToWorld( lhs, rhs ); }
		else
		{
			throw std::runtime_error( "PoseNoise: Invalid frame." );
		}
	}

	static PoseNoise ComposeToBody( const PoseNoise& lhs, const PoseNoise& rhs )
	{
		if( lhs.IsWorld() ) { return ComposeToBody( lhs.ToBody(), rhs ); }
		if( rhs.IsWorld() ) { return ComposeToBody( lhs, rhs.ToBody() ); }
		return PoseNoise( lhs._pose * rhs._pose,
			              WorldCovToBody( lhs._cov, rhs._pose ) + rhs._cov,
			              BodyNoiseFrame );
	}

	static PoseNoise ComposeToWorld( const PoseNoise& lhs, const PoseNoise& rhs )
	{
		if( lhs.IsBody() ) { return ComposeToWorld( lhs.ToWorld(), rhs ); }
		if( rhs.IsBody() ) { return ComposeToWorld( lhs, rhs.ToWorld() ); }
		return PoseNoise( lhs._pose * rhs._pose,
			              lhs._cov + BodyCovToWorld( rhs._cov, lhs._pose ),
			              WorldNoiseFrame );
	}

private:

	PoseType _pose;
	FixedMatrixType<NoiseDim, NoiseDim> _cov;
	PoseNoiseFrame _frame;

	/*! \brief Move noise from left to right with inverse adjoint */
	static CovarianceType WorldCovToBody( const CovarianceType& cov, const PoseType& pose )
	{
		// TODO Use a solver instead of inverse?
		typename PoseType::AdjointMatrix adjinv = PoseType::Adjoint( pose ).inverse();
		return adjinv * cov * adjinv.transpose();
	}

	/*! \brief Move noise from right to left with adjoint */
	static CovarianceType BodyCovToWorld( const CovarianceType& cov, const PoseType& pose )
	{
		typename PoseType::AdjointMatrix adj = PoseType::Adjoint( pose );
		return adj * cov * adj.transpose();
	}

};

}
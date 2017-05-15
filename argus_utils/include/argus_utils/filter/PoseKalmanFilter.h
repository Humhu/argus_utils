#pragma once

#include "argus_utils/filter/FilterInfo.h"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{

template <typename Pose>
class PoseKalmanFilter
{
public:

	typedef Pose PoseType;
	typedef typename PoseType::TangentVector PoseTangent;
	typedef typename PoseType::CovarianceMatrix PoseCovariance;

	PoseKalmanFilter();
	void Initialize( const PoseType& x, const PoseCovariance& P );
	void SetTransitionCovariance( const PoseCovariance& Q );
	// TODO Support position, orientation observations
	void SetObservationCovariance( const PoseCovariance& R );

	PredictInfo Predict();
	PredictInfo Predict( const PoseType& displacement );
	PredictInfo Predict( const PoseCovariance& Q );
	PredictInfo Predict( const PoseType& displacement, const PoseCovariance& Q );

	UpdateInfo Update( const PoseType& obs );
	UpdateInfo Update( const PoseType& obs, const PoseCovariance& R );

	const PoseType& GetState() const;
	const PoseCovariance& GetCovariance() const;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:

	PoseType _x;
	PoseCovariance _P;

	PoseCovariance _Q;
	PoseCovariance _R;
};

typedef PoseKalmanFilter<PoseSE3> PoseSE3KalmanFilter;
typedef PoseKalmanFilter<PoseSE2> PoseSE2KalmanFilter;

}

#include "argus_utils/filter/PoseKalmanFilter.hpp"
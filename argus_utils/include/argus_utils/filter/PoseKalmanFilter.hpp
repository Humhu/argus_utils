#pragma once

namespace argus
{
template <typename Pose>
PoseKalmanFilter<Pose>::PoseKalmanFilter()
{
	_P = PoseCovariance::Identity();
	_Q = PoseCovariance::Identity();
	_R = PoseCovariance::Identity();
}

template <typename Pose>
void PoseKalmanFilter<Pose>::Initialize( const PoseType& x,
                                         const PoseCovariance& P )
{
	_x = x;
	_P = P;
}

template <typename Pose>
void PoseKalmanFilter<Pose>::SetTransitionCovariance( const PoseCovariance& Q )
{
	_Q = Q;
}

template <typename Pose>
void PoseKalmanFilter<Pose>::SetObservationCovariance( const PoseCovariance& R )
{
	_R = R;
}

template <typename Pose>
PredictInfo PoseKalmanFilter<Pose>::Predict()
{
	return Predict( Pose(), _Q );
}

template <typename Pose>
PredictInfo PoseKalmanFilter<Pose>::Predict( const PoseType& displacement )
{
	return Predict( displacement, _Q );
}

template <typename Pose>
PredictInfo PoseKalmanFilter<Pose>::Predict( const PoseCovariance& Q )
{
	return Predict( Pose(), Q );
}

template <typename Pose>
PredictInfo PoseKalmanFilter<Pose>::Predict( const PoseType& displacement,
                                             const PoseCovariance& Q )
{
	PredictInfo info;
	info.prior_state_cov = _P;
	info.trans_noise_cov = Q;

	// We assume that noise is maintained on the RHS, so we have to transform the
	// estimate noise on the LHS of displacement to its RHS using its 
	// inverse adjoint
	MatrixType A = Pose::Adjoint( displacement.Inverse() );
	_x = _x * displacement;
	_P = A * _P * A.transpose() + Q;

	info.trans_jacobian = A;
	info.post_state_cov = _P;
	return info;
}

template <typename Pose>
UpdateInfo PoseKalmanFilter<Pose>::Update( const PoseType& obs )
{
	return Update( obs, _R );
}

template <typename Pose>
UpdateInfo PoseKalmanFilter<Pose>::Update( const PoseType& obs,
                                           const PoseCovariance& R )
{
	UpdateInfo info;
	info.prior_state_cov = _P;

	// NOTE C = Identity
	PoseTangent v = Pose::Log( _x.Inverse() * obs );
	PoseCovariance V = _P + R;
	Eigen::LLT<PoseCovariance> Vinv( V );
	PoseCovariance K = Vinv.solve( _P ).transpose();

	_x = _x * Pose::Exp( K * v );

	PoseCovariance l = PoseCovariance::Identity() - K;
	_P = l * _P * l.transpose() + K * R * K.transpose();

	info.prior_obs_error = v;
	info.obs_error_cov = V;
	info.post_state_cov = _P;
	info.state_delta = K * v;
	info.post_obs_error = Pose::Log( _x.Inverse() * obs );
	info.kalman_gain = K;
	info.obs_jacobian = PoseCovariance::Identity();
	info.obs_noise_cov = R;
	return info;
}

template <typename Pose>
const typename PoseKalmanFilter<Pose>::PoseType&
PoseKalmanFilter<Pose>::GetState() const
{
	return _x;
}

template <typename Pose>
const typename PoseKalmanFilter<Pose>::PoseCovariance&
PoseKalmanFilter<Pose>::GetCovariance() const
{
	return _P;
}
}
#include "argus_utils/filter/KalmanFilter.h"

#include <Eigen/Cholesky>

namespace argus
{
KalmanFilter::KalmanFilter() {}

void KalmanFilter::Initialize( const VectorType& x, const MatrixType& P )
{
	_x = x;
	_P = P;
}

void KalmanFilter::SetTransitionMatrix( const MatrixType& A )
{
	_A = A;
}

void KalmanFilter::SetTransitionCovariance( const MatrixType& Q )
{
	_Q = Q;
}

void KalmanFilter::SetObservationMatrix( const MatrixType& C )
{
	_C = C;
}

void KalmanFilter::SetObservationCovariance( const MatrixType& R )
{
	_R = R;
}

PredictInfo KalmanFilter::Predict()
{
	return Predict( VectorType::Zero( _x.size() ), _A, _Q );
}

PredictInfo KalmanFilter::Predict( const MatrixType& A, const MatrixType& Q )
{
	return Predict( VectorType::Zero( _x.size() ), A, Q );
}

PredictInfo KalmanFilter::Predict( const VectorType& dx )
{
	return Predict( dx, _A, _Q );
}

PredictInfo KalmanFilter::Predict( const VectorType& dx, const MatrixType& A,
                                   const MatrixType& Q )
{
	PredictInfo info;
	info.prior_state_cov = _P;
	info.trans_jacobian = A;
	info.trans_noise_cov = Q;

	_x = A * _x + dx;
	_P = (A * _P * A.transpose()).eval() + Q;

	info.post_state_cov = _P;
	return info;
}

UpdateInfo KalmanFilter::Update( const VectorType& y )
{
	return Update( y, _C, _R );
}

UpdateInfo KalmanFilter::Update( const VectorType& y, const MatrixType& C,
                                 const MatrixType& R )
{
	UpdateInfo info;
	info.prior_state_cov = _P;
	info.obs = y;

	VectorType v = y - C * _x;
	MatrixType V = C * _P * C.transpose() + R;
	Eigen::LLT<MatrixType> Vinv( V );
	MatrixType K = Vinv.solve( C * _P ).transpose();
	_x = _x + K * v;
	// Joseph form of the update for stability
	MatrixType l = MatrixType::Identity( _x.size(), _x.size() ) - K * C;
	_P = (l * _P * l.transpose() + K * R * K.transpose()).eval();
	
	info.prior_obs_error = v;
	info.obs_error_cov = V;
	info.post_state_cov = _P;
	info.state_delta = K * v;
	info.post_obs_error = y - C * _x;
	info.kalman_gain= K;
	info.obs_jacobian = C;
	info.obs_noise_cov = R;
	return info;
}

const VectorType& KalmanFilter::GetState() const { return _x; }
const MatrixType& KalmanFilter::GetCovariance() const { return _P; }
}
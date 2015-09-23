#pragma once

#include <Eigen/Dense>

namespace argus_utils 
{

/*! \class KalmanFilter KalmanFilter.h
* \brief A basic discrete-time Kalman filter with compile-time size. */
template < unsigned int N, class T = double >
class KalmanFilter 
{
public:

	typedef Eigen::Matrix<T, N, 1> StateVector;
	typedef Eigen::Matrix<T, N, N> StateCovariance;
	typedef Eigen::Matrix<T, N, N> StateTransition;

	typedef Eigen::Matrix<T, Eigen::Dynamic, 1> MeasurementVector;
	typedef Eigen::Matrix<T, Eigen::Dynamic, N> MeasurementMatrix;
	typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MeasurementCovariance;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	KalmanFilter() {}

	StateTransition& TransitionMatrix() { return A; }
	StateCovariance& TransitionCovariance() { return Q; }
	MeasurementMatrix& ObservationMatrix() { return C; }
	MeasurementCovariance& ObservationCovariance() { return R; }
	StateVector& EstimateMean() { return x; }
	StateCovariance& EstimateCovariance() { return S; }
	
	/*! \brief Execute a predict step. */
	virtual void Predict() { Predict( Q ); };

	/*! \brief Execute a predict step with prescribed covariance. */
	virtual void Predict( const StateCovariance& q ) 
	{
		x = A*x;
		S = A*S*A.transpose() + q;
	}

	/*! \brief Execute a measurement update step. */
	virtual void Update( const MeasurementVector& z ) { Update( z, R ); }

	/*! \brief Execute a measurement update with a prescribed covariance. */
	virtual void Update( const MeasurementVector& z, const MeasurementCovariance& r )
	{
		MeasurementVector yres = z - C*x;
		MeasurementCovariance Sres = C*S*C.transpose() + r;
		Eigen::ColPivHouseholderQR<MeasurementCovariance> dec( Sres.transpose() );
		MeasurementMatrix K = dec.solve( C*S.transpose() ).transpose();
		x = x + K*yres;
		S = ( StateCovariance::Identity() - K*C )*S;
	}

protected:

	/*! \brief The current state estimate mean. */
	StateVector x;

	/*! \brief The current state estimate covariance. */
	StateCovariance S;

	/*! \brief The state transition matrix. */
	StateTransition A;

	/*! \brief The state transition covariance matrix. */
	StateCovariance Q;

	/*! \brief The state observation matrix. */
	MeasurementMatrix C;

	/*! \brief The observation covariance matrix. */
	MeasurementCovariance R;

};

} // end namespace argus_utils

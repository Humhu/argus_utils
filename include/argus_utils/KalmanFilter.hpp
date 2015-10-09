#pragma once

#include <Eigen/Dense>

namespace argus_utils 
{

/*! \class KalmanFilter KalmanFilter.h
* \brief A basic discrete-time Kalman filter with compile-time sizes. */
template < typename Scalar = double,
		   int StateDim = Eigen::Dynamic,
		   int ControlDim = Eigen::Dynamic,
		   int ObsDim = Eigen::Dynamic >
class KalmanFilter 
{
public:

	typedef Eigen::Matrix<Scalar, StateDim, 1> 			StateVector;
	typedef Eigen::Matrix<Scalar, StateDim, StateDim> 	StateCovariance;
	typedef Eigen::Matrix<Scalar, StateDim, StateDim> 	StateTransition;
	
	typedef Eigen::Matrix<Scalar, ControlDim, 1>		ControlVector;
	typedef Eigen::Matrix<Scalar, StateDim, ControlDim> ControlTransition;

	typedef Eigen::Matrix<Scalar, ObsDim, 1> 			ObservationVector;
	typedef Eigen::Matrix<Scalar, ObsDim, StateDim> 	ObservationMatrix;
	typedef Eigen::Matrix<Scalar, ObsDim, ObsDim> 		ObservationCovariance;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	KalmanFilter() {}

	StateTransition& TransMatrix() { return A; }
	StateCovariance& TransCovariance() { return Q; }
	
	ControlTransition& ControlMatrix() { return B; }
	
	ObservationMatrix& ObsMatrix() { return C; }
	ObservationCovariance& ObsCovariance() { return R; }
	
	StateVector& EstimateMean() { return x; }
	StateCovariance& EstimateCovariance() { return S; }
	
	/*! \brief Execute a predict step with no controls. */
	virtual void Predict() { Predict( Q ); };

	/*! \brief Execute a predict step with no controls and prescribed covariance. */
	virtual void Predict( const StateCovariance& q ) 
	{
		x = A*x;
		S = A*S*A.transpose() + q;
	}

	/*! \brief Execute a predict step with controls. */
	virtual void Predict( const ControlVector& u )
	{
		Predict( u, Q );
	}
	
	/*! \brief Execute a predict step with controls and prescribed covariance. */
	virtual void Predict( const ControlVector& u, const StateCovariance& q )
	{
		Predict();
		x += B*u;
	}
	
	/*! \brief Execute a measurement update step. */
	virtual void Update( const ObservationVector& z ) { Update( z, R ); }

	/*! \brief Execute a measurement update with a prescribed covariance. */
	virtual void Update( const ObservationVector& z, const ObservationCovariance& r )
	{
		ObservationVector yres = z - C*x;
		ObservationCovariance Sres = C*S*C.transpose() + r;
		Eigen::ColPivHouseholderQR<ObservationCovariance> dec( Sres.transpose() );
		ObservationMatrix K = dec.solve( C*S.transpose() ).transpose();
		x = x + K*yres;
		S = ( StateCovariance::Identity() - K*C )*S;
	}

protected:

	/*! \brief The current state estimate mean and covariance. */
	StateVector x;
	StateCovariance S;

	/*! \brief The state transition matrices. x evolves as x' = Ax + Bu */
	StateTransition A;
	ControlTransition B;

	/*! \brief The state transition covariance matrix. */
	StateCovariance Q;

	/*! \brief The state observation matrix. */
	ObservationMatrix C;

	/*! \brief The observation covariance matrix. */
	ObservationCovariance R;

};

} // end namespace argus_utils

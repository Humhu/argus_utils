#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>

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

	typedef Eigen::Matrix<Scalar, StateDim, 1>          StateVector;
	typedef Eigen::Matrix<Scalar, StateDim, StateDim>   StateCovariance;
	typedef Eigen::Matrix<Scalar, StateDim, StateDim>   StateTransition;
	
	typedef Eigen::Matrix<Scalar, ControlDim, 1>        ControlVector;
	typedef Eigen::Matrix<Scalar, StateDim, ControlDim> ControlTransition;

	typedef Eigen::Matrix<Scalar, ObsDim, 1>            ObservationVector;
	typedef Eigen::Matrix<Scalar, ObsDim, StateDim>     ObservationMatrix;
	typedef Eigen::Matrix<Scalar, ObsDim, ObsDim>       ObservationCovariance;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	KalmanFilter() {}

	StateTransition& TransMatrix() { return A; }
	const StateTransition& TransMatrix() const { return A; }
	StateCovariance& TransCovariance() { return Q; }
	const StateCovariance& TransCovariance() const { return Q; }
	
	ControlTransition& ControlMatrix() { return B; }
	const ControlTransition& ControlMatrix() const { return B; }
	
	ObservationMatrix& ObsMatrix() { return C; }
	const ObservationMatrix& ObsMatrix() const { return C; }
	ObservationCovariance& ObsCovariance() { return R; }
	const ObservationCovariance& ObsCovariance() const { return R; }
	
	StateVector& EstimateMean() { return x; }
	const StateVector& EstimateMean() const { return x; }
	StateCovariance& EstimateCovariance() { return S; }
	const StateCovariance& EstimateCovariance() const { return S; }
	
	/*! \brief Execute a predict step with no controls. */
	void Predict() { Predict( Q ); };

	/*! \brief Execute a predict step with no controls and prescribed covariance. */
	void Predict( const StateCovariance& q ) 
	{
		x = A*x;
		S = A*S*A.transpose() + q;
	}

	/*! \brief Execute a predict step with controls. */
	void PredictControl( const ControlVector& u )
	{
		PredictControl( u, Q );
	}
	
	/*! \brief Execute a predict step with controls and prescribed covariance. */
	void PredictControl( const ControlVector& u, const StateCovariance& q )
	{
		Predict();
		x += B*u;
	}
	
	/*! \brief Execute a measurement update step. */
	void Update( const ObservationVector& z ) { Update( z, R ); }

	/*! \brief Execute a measurement update with a prescribed covariance. */
	void Update( const ObservationVector& z, const ObservationCovariance& r )
	{
		ObservationVector v = z - C*x;
		ObservationCovariance V = C*S*C.transpose() + r;
		Eigen::LLT<ObservationCovariance> dec( V );
		ObservationMatrix K = dec.solve( C*S.transpose() ).transpose();
		x = x + K*v;
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

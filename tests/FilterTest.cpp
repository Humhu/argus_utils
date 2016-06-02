#include "argus_utils/filters/KalmanFilter.hpp"
#include "argus_utils/filters/ExtendedKalmanFilter.hpp"
#include "argus_utils/filters/PoseKalmanFilter.hpp"
#include "argus_utils/filters/DerivativePoseFilter.hpp"
#include "argus_utils/geometry/PoseNoise.hpp"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

#include <boost/bind.hpp>
#include <iostream>

using namespace argus;

typedef ExtendedKalmanFilter<3, 3, 3> EKF;
typedef PoseKalmanFilter<PoseSE3> SE3KF;
typedef DerivativePoseFilter<PoseSE3,2> DSE3KF;

EKF::StateVector f( const EKF::StateVector& x, const EKF::ControlVector& u )
{
	return x - u;
}

EKF::TransitionJacobian F( const EKF::StateVector& x, const EKF::ControlVector& u )
{
	return EKF::TransitionJacobian::Identity();
}

EKF::ObservationVector h( const EKF::StateVector& x )
{
	return x;
}

EKF::ObservationJacobian H( const EKF::StateVector& x )
{
	return EKF::ObservationJacobian::Identity();
}

void TestKF() {
	
	// Fixed dim 3 state, dynamic control and obs dims
	typedef KalmanFilter<3, 3, 3> KF;
	
	KF kf;
	kf.TransMatrix() = KF::StateTransition::Identity();
	kf.TransCovariance() = KF::StateCovariance::Identity();
	
	kf.ControlMatrix() = -KF::ControlTransition::Identity();
	
	kf.ObsMatrix() = KF::ObservationMatrix::Identity();
	kf.ObsCovariance() = KF::ObservationCovariance::Identity();
	kf.EstimateMean() = KF::StateVector::Zero();
	kf.EstimateCovariance() = KF::StateCovariance::Identity();
	
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;
	
	std::cout << "Predicting..." << std::endl;
	KF::ControlVector u;
	u << 0, -1, 1;
	kf.PredictControl( u );
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;

	std::cout << "Updating..." << std::endl;	
	KF::ObservationVector z = KF::ObservationVector::Zero();
	z(2) = 1;
	kf.Update( z );
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;
	
}

void TestEKF() {
	// EKF test
	EKF ekf;
	ekf.TransFunction() = boost::bind( &f, _1, _2 );
	ekf.TransJacFunction() = boost::bind( &F, _1, _2 );
	ekf.TransCovariance() = EKF::StateCovariance::Identity();
	ekf.ObsFunction() = boost::bind( &h, _1 );
	ekf.ObsJacFunction() = boost::bind( &H, _1 );
	ekf.ObsCovariance() = EKF::ObservationCovariance::Identity();
	
	ekf.EstimateMean() = EKF::StateVector::Zero();
	ekf.EstimateCovariance() = EKF::StateCovariance::Identity();
	
	std::cout << "EKF is at mean: " << ekf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << ekf.EstimateCovariance() << std::endl;
	
	std::cout << "Predicting..." << std::endl;
	EKF::ControlVector u;
	u << 0, -1, 1;
	ekf.Predict( u );
	std::cout << "EKF is at mean: " << ekf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << ekf.EstimateCovariance() << std::endl;

	std::cout << "Updating..." << std::endl;	
	EKF::ObservationVector z = EKF::ObservationVector::Zero();
	z(2) = 1;
	ekf.Update( z );
	std::cout << "EKF is at mean: " << ekf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << ekf.EstimateCovariance() << std::endl;
	
}

void TestSE3KF( PoseNoiseFrame frame )
{
	typedef PoseSE3 PoseType;
	typedef PoseSE3::CovarianceMatrix CovarianceType;
	typedef PoseNoise<PoseType> PoseNoiseType;
	
	PoseType initPose( 0, 1, 0, 1, 0, 0, 0 );
	CovarianceType initCov = CovarianceType::Identity();
	PoseNoiseType init( initPose, initCov, frame );

	SE3KF se3kf( init );
	std::cout << "Initialized: " << std::endl;
	std::cout << "mean: " << se3kf.Estimate().Pose() << std::endl;
	std::cout << "cov: " << std::endl << se3kf.Estimate().Covariance() << std::endl;

	PoseType dispPose( 0.1, 0, 0, 1, 0, 0, 0.1 ); // Move forward and yaw a little
	CovarianceType dispCov = 0.1 * CovarianceType::Identity();
	dispCov(0,0) = 10; dispCov(5,5) = 10;
	PoseNoiseType disp( dispPose, dispCov, frame );
	se3kf.PredictBodyDisplace( disp );
	std::cout << "Body displaced: " << std::endl;
	std::cout << "mean: " << se3kf.Estimate().Pose() << std::endl;
	std::cout << "cov: " << std::endl << se3kf.Estimate().Covariance() << std::endl;

	se3kf.Estimate() = init;
	se3kf.PredictWorldDisplace( disp );
	std::cout << "Reinitialized, World displaced: " << std::endl;
	std::cout << "mean: " << se3kf.Estimate().Pose() << std::endl;
	std::cout << "cov: " << std::endl << se3kf.Estimate().Covariance() << std::endl;

	se3kf.Estimate() = init;
	PoseType obsPose( 0.1, 1, 0, 1, 0, 0, 0 );
	CovarianceType obsCov = 0.1 * CovarianceType::Identity();
	obsCov(0,0) = 10; obsCov(5,5) = 10;
	PoseNoiseType obs( obsPose, obsCov, frame );
	se3kf.Update( obs );
	std::cout << "Reinitialized, updated: " << std::endl;
	std::cout << "mean: " << se3kf.Estimate().Pose() << std::endl;
	std::cout << "cov: " << std::endl << se3kf.Estimate().Covariance() << std::endl;
}

void TestDSE3KF()
{
	typedef PoseSE3 PoseType;
	typedef DSE3KF::DerivsType DerivType;
	typedef DSE3KF::FullCovType CovType;

	PoseType initPose( 0, 1, 0, 1, 0, 0, 0 );
	DerivType initDeriv;
	initDeriv << 0, 0, 0, 0, 0, 0;
	CovType initCov = CovType::Identity();

	DSE3KF filter( initPose, initDeriv, initCov );
	MatrixType A = filter.TransitionMatrixFunc()(0.1);
	std::cout << "A: " << std::endl << A << std::endl;

	VectorType accObs(6);
	accObs << 0.3, 0, 0, 0, 0, 0.5;
	DSE3KF::DerivObsMatrix accC = DSE3KF::DerivObsMatrix::Zero(6,DSE3KF::DerivsDim);
	accC.rightCols(6) = FixedMatrixType<6,6>::Identity();
	MatrixType accR = 0.1 * MatrixType::Identity(6,6);
	filter.UpdateDerivs( accObs, accC, accR );
	std::cout << "Acc Update" << std::endl;
	std::cout << "Pose: " << filter.Pose() << std::endl;
	std::cout << "Derivs: " << filter.Derivs().transpose() << std::endl;
	std::cout << "Cov: " << std::endl << filter.FullCov() << std::endl;

	double dt = 0.1;
	DSE3KF::FullCovType Qrate = DSE3KF::FullCovType::Identity();
	for( unsigned int i = 0; i < 4; i++ )
	{
		filter.Predict( dt*Qrate, dt );
		std::cout << "Predict " << i << std::endl;
		std::cout << "Pose: " << filter.Pose() << std::endl;
		std::cout << "Derivs: " << filter.Derivs().transpose() << std::endl;
		std::cout << "Cov: " << std::endl << filter.FullCov() << std::endl;
	}

	VectorType velObs(6);
	velObs << 0, 0, 0, 0, 0, 0;
	DSE3KF::DerivObsMatrix velC = DSE3KF::DerivObsMatrix::Zero(6,DSE3KF::DerivsDim);
	velC.leftCols(6) = FixedMatrixType<6,6>::Identity();
	MatrixType velR = 0.1 * MatrixType::Identity(6,6);
	filter.UpdateDerivs( velObs, velC, velR );
	std::cout << "Vel Update" << std::endl;
	std::cout << "Pose: " << filter.Pose() << std::endl;
	std::cout << "Derivs: " << filter.Derivs().transpose() << std::endl;
	std::cout << "Cov: " << std::endl << filter.FullCov() << std::endl;
}

int main( int argc, char** argv )
{
	TestKF();
	TestEKF();

	TestSE3KF( BodyNoiseFrame );
	TestDSE3KF();
}


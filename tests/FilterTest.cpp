#include "argus_utils/KalmanFilter.hpp"
#include "argus_utils/ExtendedKalmanFilter.hpp"
#include "argus_utils/ManifoldKalmanFilter.hpp"
#include "argus_utils/PoseSE2.h"
#include "argus_utils/PoseSE3.h"

#include <boost/bind.hpp>
#include <iostream>

using namespace argus_utils;

typedef ExtendedKalmanFilter<double, 3, 3, 3> EKF;

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
	typedef KalmanFilter<double, 3, 3, 3> KF;
	
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

void TestMKFSE2()
{
	typedef ManifoldKalmanFilter< PoseSE2, BodyFrame > BodyMKF;
// 	typedef ManifoldKalmanFilter< PoseSE2, WorldFrame > WorldMKF;
	
	BodyMKF bmkf;
	bmkf.TransCovariance() = BodyMKF::StateCovariance::Identity();
	bmkf.ObsCovariance() = BodyMKF::ObservationCovariance::Identity();
	bmkf.EstimateMean() = PoseSE2( 1, 0, 0 );
	bmkf.EstimateCovariance() = BodyMKF::StateCovariance::Identity();
	
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	BodyMKF::StateCovariance q = BodyMKF::StateCovariance::Identity();
	std::cout << "Predicting in body frame" << std::endl;
	bmkf.Predict( q, BodyFrame );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	std::cout << "Predicting in world frame" << std::endl;
	bmkf.Predict( q, WorldFrame );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	BodyMKF::ObservationCovariance r = BodyMKF::ObservationCovariance::Identity();
	PoseSE2 obs;
	std::cout << "Updating in body frame" << std::endl;
	bmkf.UpdateBody( obs );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	std::cout << "Updating in world frame" << std::endl;
	bmkf.UpdateWorld( obs );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
}

void TestMKFSE3()
{
	typedef ManifoldKalmanFilter< PoseSE3, BodyFrame > BodyMKF;
// 	typedef ManifoldKalmanFilter< PoseSE3, WorldFrame > WorldMKF;
	
	BodyMKF bmkf;
	bmkf.TransCovariance() = BodyMKF::StateCovariance::Identity();
	bmkf.ObsCovariance() = BodyMKF::ObservationCovariance::Identity();
	bmkf.EstimateMean() = PoseSE3( 1, 0, 0, 1, 0, 0, 0 );
	bmkf.EstimateCovariance() = BodyMKF::StateCovariance::Identity();
	
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	BodyMKF::StateCovariance q = BodyMKF::StateCovariance::Identity();
	std::cout << "Predicting in body frame" << std::endl;
	bmkf.Predict( q, BodyFrame );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	std::cout << "Predicting in world frame" << std::endl;
	bmkf.Predict( q, WorldFrame );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	BodyMKF::ObservationCovariance r = BodyMKF::ObservationCovariance::Identity();
	PoseSE3 obs;
	std::cout << "Updating in body frame" << std::endl;
	bmkf.UpdateBody( obs );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
	
	std::cout << "Updating in world frame" << std::endl;
	bmkf.UpdateWorld( obs );
	std::cout << "BMKF is at mean: " << bmkf.EstimateMean() << " with covariance: " << std::endl;
	std::cout << bmkf.EstimateCovariance() << std::endl;
}

int main( int argc, char** argv )
{
	TestKF();
	TestEKF();
	TestMKFSE2();
	TestMKFSE3();
}


#include "argus_utils/KalmanFilter.hpp"

#include <iostream>

using namespace argus_utils;

int main( int argc, char** argv )
{
	
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
	kf.Predict( u );
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;

	std::cout << "Updating..." << std::endl;	
	KF::ObservationVector z = KF::ObservationVector::Zero();
	z(2) = 1;
	kf.Update( z );
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;
	
}

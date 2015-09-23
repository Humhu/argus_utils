#include "argus_utils/KalmanFilter.hpp"

#include <iostream>

using namespace argus_utils;

int main( int argc, char** argv )
{
	
	typedef KalmanFilter<3> KF;
	
	KF kf;
	kf.TransitionMatrix() = KF::StateTransition::Identity();
	kf.TransitionCovariance() = KF::StateCovariance::Identity();
	kf.ObservationMatrix() = KF::MeasurementMatrix::Identity(3,3);
	kf.ObservationCovariance() = KF::MeasurementCovariance::Identity(3,3);
	kf.EstimateMean() = KF::StateVector::Zero();
	kf.EstimateCovariance() = KF::StateCovariance::Identity();
	
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;
	
	std::cout << "Predicting..." << std::endl;
	kf.Predict();
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;

	std::cout << "Updating..." << std::endl;	
	KF::MeasurementVector z = KF::MeasurementVector::Zero(3);
	z(2) = 1;
	kf.Update( z );
	std::cout << "KF is at mean: " << kf.EstimateMean().transpose() << " with covariance: " << std::endl;
	std::cout << kf.EstimateCovariance() << std::endl;
	
}

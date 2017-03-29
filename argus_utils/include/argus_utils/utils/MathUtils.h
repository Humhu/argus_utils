#pragma once

#include <vector>
#include <cstdlib>

namespace argus
{

// Performs long-division of integer input into a number system with specified bases
// at each significant place
std::vector<unsigned int> 
multibase_long_div( unsigned int input, const std::vector<unsigned int>& bases )
{
	unsigned int power = 1;
	for( unsigned int i = 1; i < bases.size(); ++i )
	{
		power *= bases[i];
	}

	std::vector<unsigned int> inputs;
	std::div_t result;
	for( unsigned int i = 0; i < bases.size()-1; ++i )
	{
		result = std::div( (int) input, (int) power );
		inputs.push_back( result.quot );
		input = result.rem;
		power = power / bases[i+1];
	}
	inputs.push_back( input );
	return inputs;
}

/*! \brief Construct a discrete-time integrator matrix with the
 * specified number of derivatives*/
template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>
IntegralMatrix( double dt, unsigned int order )
{
	typedef Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> Mat;
	unsigned int N = PoseSE3::TangentDimension * (order + 1);
	Mat mat = Mat::Identity( N, N );

	double intTerm = 1.0;
	for( unsigned int o = 1; o <= order; ++o )
	{
		intTerm = intTerm * dt / o;
		for( int i = 0; i < PoseSE3::TangentDimension*(order-o+1); ++i )
		{
			mat( i, i + PoseSE3::TangentDimension*o ) = intTerm;
		}
	}
	return mat;
}

}
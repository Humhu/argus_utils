#pragma once

#include <vector>
#include <cstdlib>
#include <Eigen/Dense>

namespace argus
{

// Performs long-division of integer input into a number system with specified bases
// at each significant place
std::vector<unsigned int> 
multibase_long_div( unsigned int input, const std::vector<unsigned int>& bases );

/*! \brief Construct a discrete-time integrator matrix with the
 * specified number of derivatives*/
template <typename S>
Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic>
IntegralMatrix( double dt, unsigned int dim, unsigned int order )
{
	typedef Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> Mat;
	unsigned int N = dim * (order + 1);
	Mat mat = Mat::Identity( N, N );

	double intTerm = 1.0;
	for( unsigned int o = 1; o <= order; ++o )
	{
		intTerm = intTerm * dt / o;
		for( int i = 0; i < dim*(order-o+1); ++i )
		{
			mat( i, i + dim*o ) = intTerm;
		}
	}
	return mat;
}

}
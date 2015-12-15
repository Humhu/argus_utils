#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <boost/random/random_device.hpp>
#include <boost/random/random_number_generator.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

namespace argus_utils
{

// TODO Generalize multivariate normal and this into some adapter class?
	
/*! \brief Takes uniform samples from a unit sphere and converts them to an ellipse. */
template <int N, typename Engine = boost::mt19937, typename Scalar = double>
class UniformEllipse 
{
public:
	
	typedef Scalar ScalarType;
	typedef Eigen::Matrix<Scalar, N, 1> VectorType;
	typedef Eigen::Matrix<Scalar, N, N> MatrixType;
	
	typedef boost::uniform_real_distribution<Scalar> UniformReal;
	
	UniformEllipse()
	: distribution( -1.0, 1.0 ),
	  mean( VectorType::Zero() ),
	  L( MatrixType::Identity() )
	{
		boost::random::random_device rng;
		generator.seed( rng );
	}
	
	UniformEllipse( unsigned long seed )
	: distribution( -1.0, 1.0 ),
	  mean( VectorType::Zero() ),
	  L( MatrixType::Identity() )
	{
		generator.seed( seed );
	}
	
	UniformEllipse( const VectorType& u, const MatrixType& S )
	: distribution( -1.0, 1.0 ), 
	  mean( u ),
	  L( S.llt().matrixL() )
	{
		boost::random::random_device rng;
		generator.seed( rng );
	}
	
	UniformEllipse( const VectorType& u, const MatrixType& S, unsigned long seed )
	: distribution( -1.0, 1.0 ),
	  mean( u ),
	  L( S.llt().matrixL() )
	{
		generator.seed( seed );
	}

	void SetMean( const VectorType& u ) { mean = u; }
		
	/*! \brief Set the ellipse covariance matrix. */
	void SetEllipse( const MatrixType& S ) { L = S.llt().matrixL(); }
	
	VectorType Sample()
	{
		VectorType samples;
		for( unsigned int i = 0; i < N; i++ )
		{
			samples(i) = distribution( generator );
		}
		return L * samples + u;
	}
	
	double EvaluateProbability( const VectorType& x ) const
	{
		// TODO
		return 1.0;
	}
	
private:
	
	Engine generator;
	UniformReal distribution;
	
	VectorType mean;
	MatrixType L;
};

	
} // end namespace argus_utils

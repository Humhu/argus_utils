#pragma once

#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <boost/random/random_device.hpp>
#include <boost/random/random_number_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <cassert>

namespace argus_utils 
{

/*! \brief Simple multivariate normal sampling and PDF class. */
template <int N, typename Engine = boost::mt19937, typename Scalar = double>
class MultivariateGaussian 
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Scalar ScalarType;
	typedef Eigen::Matrix<Scalar, N, 1> VectorType;
	typedef Eigen::Matrix<Scalar, N, N> MatrixType;

	typedef boost::normal_distribution<Scalar> UnivariateNormal;
	typedef boost::variate_generator<Engine&, UnivariateNormal> RandAdapter;

	/*! \brief Seeds the engine using a true random number. Sets mean to zero
	 * and covariance to identity. */
	MultivariateGaussian()
	: distribution( 0.0, 1.0 ), 
	  adapter( generator, distribution ),
	  mean( VectorType::Zero() ), covariance( MatrixType::Identity() )
	{
		boost::random::random_device rng;
		generator.seed( rng );
		Initialize();
	}
	
	/*! \brief Seeds the engine using the specified seed. Sets mean to zero
	 * and covariance to identity. */
	MultivariateGaussian( unsigned long seed )
	: distribution( 0.0, 1.0 ), 
	  adapter( generator, distribution ),
	  mean( VectorType::Zero() ), covariance( MatrixType::Identity() )
	{
		generator.seed( seed );
		Initialize();
	}
	
	/*! \brief Seeds the engine using a true random number. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S )
	: distribution( 0.0, 1.0 ), 
	  adapter( generator, distribution ),
	  mean( u ), covariance( S )
	{
		boost::random::random_device rng;
		generator.seed( rng );
		Initialize();
	}
    
	/*! \brief Seeds the engine using a specified seed. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S, unsigned long seed )
	: distribution( 0.0, 1.0 ), 
	  adapter( generator, distribution ),
	  mean( u ), covariance( S )
	{
		generator.seed( seed );
		Initialize();
	}
    
    void SetMean( const VectorType& u ) { mean = u; }
    void SetCovariance( const MatrixType& S )
	{
		covariance = S;
		Initialize();
	}
    
	const VectorType& GetMean() const { return mean; }
	const MatrixType& GetCovariance() const { return covariance; }
	const MatrixType& GetCholesky() const { return L; }
	
	/*! \brief Generate a sample truncated at a specified number of standard deviations. */
	VectorType Sample( double v = 3.0 )
	{
		VectorType samples;
		for( unsigned int i = 0; i < N; i++ )
		{
			double s;
			do
			{
				s = adapter();
			}
			while( std::abs( s ) > v );
			samples(i) = s;
		}
		
		return mean + L*samples;
	}

	/*! \brief Evaluate the multivariate normal PDF for the specified sample. */
    double EvaluateProbability( const VectorType& x ) const
	{
		VectorType diff = x - mean;
		Eigen::Matrix<Scalar, 1, 1> exponent = -0.5 * diff.transpose() * llt.solve( diff );
		return z * std::exp( exponent(0) );
	}
    
protected:
	
	Engine generator;
	UnivariateNormal distribution;
	RandAdapter adapter;

	VectorType mean;
	MatrixType covariance;

	MatrixType L;

	double z; // Normalization constant;
	Eigen::LLT<MatrixType> llt;

	void Initialize()
	{
		assert( mean.rows() == covariance.rows() );
		assert( mean.rows() == covariance.cols() );
		llt = Eigen::LLT<MatrixType>( covariance );
		L = llt.matrixL();
		z = std::pow( 2*M_PI, -N/2.0 ) * std::pow( covariance.determinant(), -0.5 );
	}

};

}

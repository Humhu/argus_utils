#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <boost/random/random_device.hpp>
#include <boost/random/random_number_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "argus_utils/utils/LinalgTypes.h"

#include <cassert>

namespace argus 
{

// TODO
inline
double GaussianPDF( const MatrixType& cov, const VectorType& x )
{
	Eigen::LDLT<MatrixType> ldlt( cov );
	MatrixType exponent = -0.5 * x.transpose() * ldlt.solve( x );
	double z = std::pow( 2*M_PI, -x.size()/2.0 )
		       * std::pow( cov.determinant(), -0.5 );
	return z * std::exp( exponent(0) );
}

/*! \brief Multivariate normal sampling and PDF class. */
template <typename Engine = boost::mt19937>
class MultivariateGaussian 
{
public:

	typedef boost::normal_distribution<double> UnivariateNormal;
	typedef boost::variate_generator<Engine&, UnivariateNormal> RandAdapter;

	/*! \brief Seeds the engine using a true random number. Sets _mean to _zero
	 * and _covariance to identity. */
	MultivariateGaussian( unsigned int dim = 1 )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( VectorType::Zero( dim ) )
	{
		boost::random::random_device rng;
		_generator.seed( rng );
		InitializeCov( MatrixType::Identity( dim, dim ) );
	}
	
	/*! \brief Seeds the engine using the specified seed. Sets _mean to _zero
	 * and _covariance to identity. */
	MultivariateGaussian( unsigned int dim, unsigned long seed )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( VectorType::Zero( dim ) )
	{
		_generator.seed( seed );
		InitializeCov( MatrixType::Identity( dim, dim ) );
	}
	
	/*! \brief Seeds the engine using a true random number. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( u )
	{
		boost::random::random_device rng;
		_generator.seed( rng );
		InitializeCov( S );
	}
    
	/*! \brief Seeds the engine using a specified seed. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S, unsigned long seed )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( u )
	{
		_generator.seed( seed );
		InitializeCov( S );
	}

	MultivariateGaussian( const MultivariateGaussian& other )
	: _generator( other._generator ),
	  _distribution( 0.0, 1.0 ),
	  _adapter( _generator, _distribution ),
	  _mean( other._mean )
	{}

	MultivariateGaussian& operator=( const MultivariateGaussian& other )
	{
		_mean = other._mean;
		_z = other._z;
		_llt = other._llt;
		_generator = other._generator;
		return *this;
	}
    
    void SetMean( const VectorType& u ) 
    { 
    	if( u.size() != _mean.size() )
    	{
    		throw std::runtime_error( "MultivariateGaussian: Invalid mean dimension." );
    	}
    	_mean = u; 
    }
    void SetCovariance( const MatrixType& S )
	{
		if( S.rows() != _llt.matrixL().rows() || S.cols() != _llt.matrixL().cols() )
		{
			throw std::runtime_error( "MultivariateGaussian: Invalid covariance dimensions." );
		}
		InitializeCov( S );
	}

	void SetInformation( const MatrixType& I )
	{
		if( I.rows() != _llt.matrixL().rows() || I.cols() != _llt.matrixL().cols() )
		{
			throw std::runtime_error( "MultivariateGaussian: Invalid information dimensions." );
		}
		Eigen::LDLT<MatrixType> llti( I );
		MatrixType cov = llti.solve( MatrixType::Identity( I.rows(), I.cols() ) );
		InitializeCov( cov );
	}
    
	const VectorType& GetMean() const { return _mean; }
	const MatrixType& GetCovariance() const { return _llt.reconstructedMatrix(); }
	const MatrixType& GetCholesky() const { return _llt.matrixL(); }
	
	/*! \brief Generate a sample truncated at a specified number of standard deviations. */
	VectorType Sample( double v = 3.0 )
	{
		VectorType samples( _mean.size() );
		for( unsigned int i = 0; i < _mean.size(); i++ )
		{
			double s;
			do
			{
				s = _adapter();
			}
			while( std::abs( s ) > v );
			samples(i) = s;
		}
		
		return _mean + _llt.matrixL()*samples;
	}

	/*! \brief Evaluate the multivariate normal PDF for the specified sample. */
    double EvaluateProbability( const VectorType& x ) const
	{
		if( x.size() != _mean.size() )
		{
			throw std::runtime_error( "MultivariateGaussian: Invalid sample dimension." );
		}
		VectorType diff = x - _mean;
		MatrixType exponent = -0.5 * diff.transpose() * _llt.solve( diff );
		return _z * std::exp( exponent(0) );
	}
    
protected:
	
	Engine _generator;
	UnivariateNormal _distribution;
	RandAdapter _adapter;

	VectorType _mean;
	double _z; // Normalization constant;
	Eigen::LLT<MatrixType> _llt;

	void InitializeCov( const MatrixType& cov )
	{
		if( _mean.size() != cov.rows() || 
		    _mean.size() != cov.cols() )
		{
			throw std::runtime_error( "MultivariateGaussian: mean and covariance dimension mismatch." );
		}

		_llt = Eigen::LLT<MatrixType>( cov );
		_z = std::pow( 2*M_PI, -_mean.size()/2.0 )
		     * std::pow( cov.determinant(), -0.5 );
	}
};

}

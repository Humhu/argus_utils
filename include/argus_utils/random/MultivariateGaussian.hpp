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

/*! \brief Multivariate normal sampling and PDF class. */
template <typename Engine = boost::mt19937>
class MultivariateGaussian 
{
public:

	typedef boost::normal_distribution<double> UnivariateNormal;
	typedef boost::variate_generator<Engine&, UnivariateNormal> RandAdapter;

	/*! \brief Seeds the engine using a true random number. Sets _mean to _zero
	 * and _covariance to identity. */
	MultivariateGaussian( unsigned int dim )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( VectorType::Zero( dim ) ), _covariance( MatrixType::Identity( dim, dim ) )
	{
		boost::random::random_device rng;
		_generator.seed( rng );
		Initialize();
	}
	
	/*! \brief Seeds the engine using the specified seed. Sets _mean to _zero
	 * and _covariance to identity. */
	MultivariateGaussian( unsigned int dim, unsigned long seed )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( VectorType::Zero( dim ) ), _covariance( MatrixType::Identity( dim, dim ) )
	{
		_generator.seed( seed );
		Initialize();
	}
	
	/*! \brief Seeds the engine using a true random number. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( u ), _covariance( S )
	{
		boost::random::random_device rng;
		_generator.seed( rng );
		Initialize();
	}
    
	/*! \brief Seeds the engine using a specified seed. */
	MultivariateGaussian( const VectorType& u, const MatrixType& S, unsigned long seed )
	: _distribution( 0.0, 1.0 ), 
	  _adapter( _generator, _distribution ),
	  _mean( u ), _covariance( S )
	{
		_generator.seed( seed );
		Initialize();
	}
    
    void SetMean( const VectorType& u ) 
    { 
    	if( u.size() != _mean.size() )
    	{
    		throw std::runtime_error( "MultivariateGaussian: Invalid _mean dimension." );
    	}
    	_mean = u; 
    }
    void SetCovariance( const MatrixType& S )
	{
		if( S.rows() != _covariance.rows() || S.cols() != _covariance.cols() )
		{
			throw std::runtime_error( "MultivariateGaussian: Invalid _covariance dimensions." );
		}
		_covariance = S;
		Initialize();
	}
    
	const VectorType& GetMean() const { return _mean; }
	const MatrixType& GetCovariance() const { return _covariance; }
	const MatrixType& GetCholesky() const { return _L; }
	
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
		
		return _mean + _L*samples;
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
	MatrixType _covariance;

	MatrixType _L;

	double _z; // Normali_zation constant;
	Eigen::LLT<MatrixType> _llt;

	void Initialize()
	{
		if( _mean.size() != _covariance.rows() || 
		    _mean.size() != _covariance.cols() )
		{
			throw std::runtime_error( "MultivariateGaussian: mean and covariance dimension mismatch." );
		}

		_llt = Eigen::LLT<MatrixType>( _covariance );
		_L = _llt.matrixL();
		_z = std::pow( 2*M_PI, -_mean.size()/2.0 )
		     * std::pow( _covariance.determinant(), -0.5 );
	}

};

}

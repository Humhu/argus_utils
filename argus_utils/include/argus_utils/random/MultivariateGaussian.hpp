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

inline
double GaussianEntropy( const MatrixType& cov )
{
	unsigned int k = cov.rows();
	Eigen::LDLT<MatrixType> ldlt( cov );
	if( ldlt.info() != Eigen::ComputationInfo::Success )
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	double det = ldlt.vectorD().array().prod();
	double base = 2 * M_PI * std::exp(1.0);
	return 0.5 * std::log( std::pow( base, k ) * det );
}

// TODO
inline
double GaussianPDF( const MatrixType& cov, const VectorType& x )
{
	Eigen::LDLT<MatrixType> ldlt( cov );
	if( ldlt.info() != Eigen::ComputationInfo::Success )
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	double exponent = -0.5 * x.dot( ldlt.solve( x ) );
	double det = ldlt.vectorD().array().prod();
	double z = 1.0 / ( std::pow( 2 * M_PI, x.size() / 2.0 )
	                   * std::sqrt( det ) );
	return z * std::exp( exponent );
}

inline
double GaussianLogPdf( const MatrixType& cov, const VectorType& x )
{
	Eigen::LDLT<MatrixType> ldlt( cov );
	if( ldlt.info() != Eigen::ComputationInfo::Success )
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	double exponent = x.dot( ldlt.solve( x ) );
	double logdet = ldlt.vectorD().array().log().sum();
	double logz = x.size() * std::log( 2 * M_PI );
	return -0.5 * (logz + logdet + exponent);
}

/*! \brief Multivariate normal sampling and PDF class. */
template<typename Engine = boost::mt19937>
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
		_logz_det = other._logz_det;
		_ldlt = other._ldlt;
		_L = other._L;
		_generator = other._generator;
		return *this;
	}

	void SetMean( const VectorType& u )
	{
		if( u.size() != _mean.size() )
		{
			throw std::invalid_argument( "MultivariateGaussian: Invalid mean dimension." );
		}
		_mean = u;
	}

	void SetCovariance( const MatrixType& S )
	{
		if( S.rows() != _L.rows() || S.cols() != _L.cols() )
		{
			throw std::invalid_argument( "MultivariateGaussian: Invalid covariance dimensions." );
		}
		InitializeCov( S );
	}

	void SetInformation( const MatrixType& I )
	{
		if( I.rows() != _L.rows() || I.cols() != _L.cols() )
		{
			throw std::invalid_argument( "MultivariateGaussian: Invalid information dimensions." );
		}
		Eigen::LDLT<MatrixType> llti( I );
		MatrixType cov = llti.solve( MatrixType::Identity( I.rows(), I.cols() ) );
		InitializeCov( cov );
	}

	unsigned int GetDimension() const { return _mean.size(); }
	const VectorType& GetMean() const { return _mean; }
	const MatrixType& GetCovariance() const { return _ldlt.reconstructedMatrix(); }
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
			samples( i ) = s;
		}

		return _mean + _L * samples;
	}

	/*! \brief Evaluate the multivariate normal PDF for the specified sample. */
	double Pdf( const VectorType& x ) const
	{
		if( x.size() != _mean.size() )
		{
			throw std::invalid_argument( "MultivariateGaussian: Invalid sample dimension." );
		}
		VectorType diff = x - _mean;
		MatrixType exponent = -0.5 * diff.transpose() * _ldlt.solve( diff );
		return _z * std::exp( exponent( 0 ) );
	}

	double LogPdf( const VectorType& x ) const
	{
		if( x.size() != _mean.size() )
		{
			throw std::invalid_argument( "MultivariateGaussian: Invalid sample dimension." );
		}
		return -0.5 * x.dot( _ldlt.solve( x ) ) + _logz_det;
	}

protected:

	Engine _generator;
	UnivariateNormal _distribution;
	RandAdapter _adapter;

	VectorType _mean;
	double _z; // Normalization constant;
	double _logz_det; // Sum of log norm constant and determinant
	Eigen::LDLT<MatrixType> _ldlt;
	MatrixType _L;

	void InitializeCov( const MatrixType& cov )
	{
		if( _mean.size() != cov.rows() ||
		    _mean.size() != cov.cols() )
		{
			throw std::invalid_argument( "MultivariateGaussian: mean and covariance dimension mismatch." );
		}

		_ldlt = Eigen::LDLT<MatrixType>( cov );
		MatrixType D = _ldlt.vectorD().asDiagonal();
		_L = _ldlt.matrixL() * D.array().sqrt().matrix();
		_z = std::pow( 2 * M_PI, -_mean.size() / 2.0 )
		     * std::pow( cov.determinant(), -0.5 );
		_logz_det = -0.5 * _mean.size() * std::log( 2 * M_PI )
		            - 0.5 * _ldlt.vectorD().array().log().sum();
	}
};
}

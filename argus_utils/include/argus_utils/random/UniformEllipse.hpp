#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <boost/random/random_device.hpp>
#include <boost/random/random_number_generator.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{
// TODO Generalize multivariate normal and this into some adapter class?

/*! \brief Takes uniform samples from a unit sphere and converts them to an ellipse. */
template <typename Engine = boost::mt19937>
class UniformEllipse
{
public:

	typedef boost::random::uniform_real_distribution<double> UniformReal;

	UniformEllipse( unsigned int dim = 1 )
		: _distribution( -1.0, 1.0 ),
		_mean( VectorType::Zero( dim ) ),
		_S( MatrixType::Identity( dim, dim ) )
	{
		Initialize();
		boost::random::random_device rng;
		_generator.seed( rng );
	}

	UniformEllipse( unsigned int dim, unsigned long seed )
		: _distribution( -1.0, 1.0 ),
		_mean( VectorType::Zero( dim ) ),
		_S( MatrixType::Identity( dim, dim ) )
	{
		Initialize();
		_generator.seed( seed );
	}

	UniformEllipse( const VectorType& u, const MatrixType& S )
		: _distribution( -1.0, 1.0 ),
		_mean( u ),
		_S( S )
	{
		Initialize();
		boost::random::random_device rng;
		_generator.seed( rng );
	}

	UniformEllipse( const VectorType& u, const MatrixType& S, unsigned long seed )
		: _distribution( -1.0, 1.0 ),
		_mean( u ),
		_S( S )
	{
		Initialize();
		_generator.seed( seed );
	}

	void SetMean( const VectorType& u )
	{
		if( u.size() != _mean.size() )
		{
			throw std::runtime_error( "UniformEllipse: Mean dimension mismatch." );
		}
		_mean = u;
	}

	/*! \brief Set the ellipse covariance matrix. */
	void SetShape( const MatrixType& S )
	{
		if( S.rows() != _L.rows() || S.cols() != _L.cols() )
		{
			throw std::runtime_error( "UniformEllipse: Shape dimension mismatch." );
		}
		_L = S.llt().matrixL();
	}

	VectorType Sample()
	{
		VectorType samples( _mean.size() );
		for( unsigned int i = 0; i < _mean.size(); i++ )
		{
			samples( i ) = _distribution( _generator );
		}
		return _L * samples + _mean;
	}

	double EvaluateProbability( const VectorType& x ) const
	{
		// TODO
		return 1.0;
	}

private:

	Engine _generator;
	UniformReal _distribution;

	VectorType _mean;
	MatrixType _S;
	MatrixType _L;
	double _z;

	void Initialize()
	{
		if( _mean.size() != _S.cols() || _mean.size() != _S.rows() )
		{
			throw std::runtime_error( "UniformEllipse: Dimension mismatch in mean and shape." );
		}
		_L = _S.llt().matrixL();
		_z = _S.determinant();
	}
};
} // end namespace argus

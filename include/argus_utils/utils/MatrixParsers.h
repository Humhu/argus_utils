#pragma once

#include "argus_utils/utils/ArrayParsers.h"
#include <Eigen/Dense>

namespace argus
{

template <typename FloatType, typename S>
bool GetFloatVector( const S& src, const std::string& name,
                     std::vector<FloatType>& vec )
{
	if( GetParam( src, name, vec ) ) { return true; }
	// If this fails, we probably have string values
	std::vector<std::string> vstr;
	if( !GetParam( src, name, vstr ) )
	{
		std::cout << "Could not get string vector." << std::endl;
		return false; 
	}
	try
	{
		for( unsigned int i = 0; i < vstr.size(); i++ )
		{
			vec.push_back( std::stod( vstr[i] ) );
		}
		return true;
	}
	// Thrown when interpretation fails
	catch( std::invalid_argument )
	{
		std::cout << "Could not interpret all string elements as double" << std::endl;
		return false;
	}
}

template <typename Derived, typename S>
bool GetMatrixParam( const S& src, const std::string& name, 
                     Eigen::DenseBase<Derived>& mat )
{
	std::vector<typename Eigen::DenseBase<Derived>::Scalar> values;
	if( !GetFloatVector( src, name, values ) ) { return false; }
	if( !ParseMatrix( values, mat ) )
	{
		ROS_WARN_STREAM( "Could not parse " << values.size() << " values from " 
		                 << name << " into " << mat.rows() << " by " << mat.cols()
		                 << " matrix." );
		return false;
	}
	return true;
}

template <typename Derived, typename S>
bool GetDiagonalParam( const S& src, const std::string& name, 
                       Eigen::DenseBase<Derived>& mat )
{
	std::vector<typename Eigen::DenseBase<Derived>::Scalar> values;
	if( !GetFloatVector( src, name, values ) ) { return false; }
	unsigned int minDim = std::min( mat.rows(), mat.cols() );
	if( values.size() != minDim ) 
	{ 
		ROS_WARN_STREAM( "Could not parse " << values.size() << " values from " 
		                 << name << " into " << minDim << " diagonal matrix." );
		return false;
	}
	
	mat.setZero();
	for( unsigned int ind = 0; ind < minDim; ++ind )
	{
		mat(ind,ind) = values[ind];
	}
	return true;
}

template <typename Derived, typename S>
bool GetParam( const S& src, const std::string& name, 
               Eigen::DenseBase<Derived>& mat )
{
	if( !GetMatrixParam<Derived>( src, name, mat ) )
	{
		if( !GetDiagonalParam<Derived>( src, name, mat ) )
		{
			ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
			return false;
		}
	}
	return true;
}

// Version of GetParam w/ default that allows default to be a different type
// This is useful because Eigen depends heavily on typecasting
template <typename Derived, typename Default, typename S>
void GetParam( const S& src, const std::string& name, 
               Eigen::DenseBase<Derived>& mat,
               const Eigen::DenseBase<Default>& def )
{
	if( !GetParam( src, name, mat ) )
	{
		ROS_WARN_STREAM( "Parameter: " << name << " will use default: " << def );
		mat = def;
	}
}

// Version of GetParamRequired with types for Eigen matrix retrieval
template <typename Derived, typename S>
void GetParamRequired( const S& src, const std::string& name, 
                       Eigen::DenseBase<Derived>& mat )
{
	if( !GetParam( src, name, mat ) )
	{
		throw std::runtime_error( "Could not retrieve required parameter: " + name );
	}
}

}
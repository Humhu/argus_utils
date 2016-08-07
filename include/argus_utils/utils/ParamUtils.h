#pragma once

#include <ros/ros.h>
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

template <typename Source>
struct FieldRetrieval
{
	template <typename T>
	static bool get( const Source& src, const std::string& field, T& data );
	static bool has( const Source& src, const std::string& field );
};

template <>
struct FieldRetrieval<ros::NodeHandle>
{
	template <typename T>
	static bool get( const ros::NodeHandle& src, 
	                 const std::string& field, 
	                 T& data )
	{
		return src.getParam( field, data );
	}

	static bool has( const ros::NodeHandle& src, const std::string& field )
	{
		return src.hasParam( field );
	}

};
typedef FieldRetrieval<ros::NodeHandle> RosFieldRetrieval;

template <>
struct FieldRetrieval<YAML::Node>
{
	template <typename T>
	static bool get( const YAML::Node& src, 
	                 const std::string& field, 
	                 T& data )
	{
		return GetYamlField( src, field, data );
	}

	static bool has( const YAML::Node& src, const std::string& field )
	{
		return src[field];
	}

};
typedef FieldRetrieval<YAML::Node> YamlFieldRetrieval;

template <typename S>
bool HasParam( const S& src, const std::string& name )
{
	return FieldRetrieval<S>::has( src, name );
}

// For all signed ints, bools, classes
template <typename T, typename S>
typename std::enable_if< (std::is_integral<T>::value && std::is_signed<T>::value) ||
                         std::is_same<T,bool>::value ||
                         !std::is_arithmetic<T>::value, bool >::type
GetParam( const S& src, const std::string& name, T& t )
{
	// if( !nh.getParam( name, t ) ) 
	if( !FieldRetrieval<S>::get( src, name, t ) )
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
		return false;
	}
	return true;
}

// For all unsigned ints
template <typename T, typename S>
typename std::enable_if< std::is_unsigned<T>::value && !std::is_same<T,bool>::value, bool >::type
GetParam( const S& src, const std::string& name, T& t )
{
	typename std::make_signed<T>::type tSigned;
	// if( !nh.getParam( name, tSigned ) ) 
	if( !FieldRetrieval<S>::get( src, name, tSigned ) )
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
		return false;
	}
	// Make sure we're above 0
	if( tSigned < 0 )
	{
		throw std::runtime_error( "Attempted to parse value " +
		                          std::to_string(tSigned) +
		                          " as unsigned." );
	}
	t = (T) tSigned;
	return true;
}

// For all floats
template <typename T, typename S>
typename std::enable_if< std::is_floating_point<T>::value, bool >::type
GetParam( const S& src, const std::string& name, T& t )
{
	// First see if normal retrieval works
	// if( !nh.getParam( name, t ) )
	if( !FieldRetrieval<S>::get( src, name, t ) )
	{
		// If not, see if it's a string that we can convert
		std::string valS;
		if( !FieldRetrieval<S>::get( src, name, valS ) ) { return false; }
		try
		{
			t = std::stod( valS );
		}
		catch( std::exception e ) 
		{
			ROS_WARN_STREAM( "Parameter " << valS << " could not be interpreted as a float." );
			return false;
		}
	}
	return true;
}

// TODO meta-templatize this one too
template <>
bool GetParam<std::vector<unsigned int>, ros::NodeHandle>( const ros::NodeHandle& src, 
                                                           const std::string& name, 
                                                           std::vector<unsigned int>& t );

template <>
bool GetParam<YAML::Node>( const ros::NodeHandle& nh, const std::string& name,
                           YAML::Node& t );

/*! \brief Retrieve a parameter from the ROS parameter server. Print an error
 * if the parameter retrieval fails. Has a specialization for unsigned ints
 * that checks to make sure the int value is > 0. */
template <typename T, typename S>
void GetParamRequired( const S& src, const std::string& name, T& t )
{
	if( !GetParam( src, name, t ) ) 
	{ 
		throw std::runtime_error( "Could not retrieve required parameter: " + name );
	}
}

/*! \brief Retreive a parameter from the ROS parameter server. Returns the
 * specified default if the parameter retrieval fails. Has a specialization
 * for unsigned ints that checks to make sure the int value is > 0. */
template <typename T, typename S>
void GetParam( const S& src, const std::string& name, T& t, 
               const T& def )
{
	if( !GetParam( src, name, t ) ) 
	{ 
		ROS_WARN_STREAM( "Parameter: " << name << " will use default: " << def );
		t = def;
	}
}

/*! \brief Get/set a parameter YAML object out of a node handle by combining calls
 * to get XmlRpc and convert it. Returns success. */
void SetYamlParam( const ros::NodeHandle& nh, const std::string& name, 
                   const YAML::Node& node );

template <typename Scalar, typename Derived, typename S>
bool GetMatrixParam( const S& src, const std::string& name, 
                     Eigen::DenseBase<Derived>& mat )
{
	std::vector<Scalar> values;
	if( !GetParam( src, name, values ) ) { return false; }
	if( !ParseMatrix( values, mat ) )
	{
		ROS_WARN_STREAM( "Could not parse values from " << name
		                 << " into " << mat.rows() << " by " << mat.cols()
		                 << " matrix." );
		return false;
	}
	return true;
}

template <typename Scalar, typename Derived, typename S>
bool GetDiagonalParam( const S& src, const std::string& name, 
                       Eigen::DenseBase<Derived>& mat )
{
	std::vector<Scalar> values;
	if( !GetParam( src, name, values ) ) { return false; }
	unsigned int minDim = std::min( mat.rows(), mat.cols() );
	if( values.size() != minDim ) 
	{ 
	  ROS_WARN_STREAM( "Could not parse values from " << name
			   << " into " << minDim << " diagonal matrix." );
	  return false;
	}
	
	mat.setZero();
	for( unsigned int ind = 0; ind < minDim; ++ind )
	{
		mat(ind,ind) = values[ind];
	}
	return true;
}

// TODO Remove the call to Scalar? It is very confusing to have GetParam<double> retrieve a matrix
template <typename Scalar, typename Derived, typename S>
bool GetParam( const S& src, const std::string& name, 
               Eigen::DenseBase<Derived>& mat )
{
	if( !GetMatrixParam<Scalar,Derived>( src, name, mat ) )
	{
		if( !GetDiagonalParam<Scalar,Derived>( src, name, mat ) )
		{
			ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
			return false;
		}
	}
	return true;
}

template <typename Scalar, typename Derived, typename Default, typename S>
void GetParam( const S& src, const std::string& name, 
               Eigen::DenseBase<Derived>& mat,
               const Eigen::DenseBase<Default>& def )
{
	if( !GetParam<Scalar>( src, name, mat ) )
	{
		ROS_WARN_STREAM( "Parameter: " << name << " will use default: " << def );
		mat = def;
	}
}

template <typename Scalar, typename Derived, typename S>
void GetParamRequired( const S& src, const std::string& name, 
                       Eigen::DenseBase<Derived>& mat )
{
	if( !GetParam<Scalar>( src, name, mat ) )
	{
		throw std::runtime_error( "Could not retrieve required parameter: " + name );
	}
}

}



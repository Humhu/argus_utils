#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace argus
{

// TODO Setter
template <typename Source>
struct FieldRetrieval
{
	template <typename T>
	static bool get( const Source& src, const std::string& field, T& data );
	template <typename T>
	static void set( Source& src, const std::string& field, const T& data );
	static bool has( const Source& src, const std::string& field );
	static std::string qualify( const Source& src );
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

	template <typename T>
	static void set( ros::NodeHandle& src,
	                 const std::string& field,
	                 const T& data )
	{
		src.setParam( field, data );
	}

	static bool has( const ros::NodeHandle& src, const std::string& field )
	{
		return src.hasParam( field );
	}

	static std::string qualify( const ros::NodeHandle& src )
	{
		return src.resolveName("");
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
		if( !src[field] ) { return false; }
		data = src[field].as<T>();
		return true;
	}

	template <typename T>
	static void set( YAML::Node& src,
	                 const std::string& field,
	                 const T& data )
	{
		src[field] = data;
	}

	static bool has( const YAML::Node& src, const std::string& field )
	{
		return src[field];
	}

	static std::string qualify( const YAML::Node& src )
	{
		YAML::Node base = src;
		std::string ret = "";
		if( base.Type() == YAML::NodeType::Map )
		{
			ret = base.as<std::string>();

		}
		return ret;
	}

};
typedef FieldRetrieval<YAML::Node> YamlFieldRetrieval;

// For all signed ints, bools
template <typename T, typename S>
typename std::enable_if< ( std::is_integral<T>::value && 
                           std::is_signed<T>::value ) ||
                           std::is_same<T,bool>::value, bool >::type
GetParam( const S& src, const std::string& name, T& t )
{
	if( !FieldRetrieval<S>::get( src, name, t ) )
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
		return false;
	}
	return true;
}

}
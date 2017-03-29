#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

MatrixType MsgToMatrix( const argus_msgs::MatrixFloat64& msg )
{
	if( msg.rows*msg.cols != msg.data.size() )
	{
		throw std::runtime_error( "MsgToMatrix: Incorrect sized matrix message." );
	}

	if( msg.column_major )
	{
		return Eigen::Map<const ColMatrixType>( msg.data.data(), msg.rows, msg.cols );
	}
	else
	{
		return Eigen::Map<const RowMatrixType>( msg.data.data(), msg.rows, msg.cols );
	}
}

argus_msgs::MatrixFloat64 MatrixToMsg( const MatrixType& mat )
{
	argus_msgs::MatrixFloat64 msg;
	msg.column_major = true;
	msg.rows = mat.rows();
	msg.cols = mat.cols();
	SerializeMatrix( mat, msg.data, ColMajor );
	return msg;
}

MatrixType MsgToSymmetric( const argus_msgs::SymmetricFloat64& msg )
{
	if( msg.dim * msg.dim != msg.data.size() )
	{
		throw std::runtime_error( "MsgToSymmetric: Incorrect sized matrix message." );
	}

	// Row and column major don't matter for symmetric
	MatrixType mat( msg.dim, msg.dim );
	unsigned int ind = 0;
	for( unsigned int i = 0; i < msg.dim; ++i )
	{
		for( unsigned int j = i; j < msg.dim; ++j )
		{
			mat(i,j) = msg.data[ind];
			mat(j,i) = msg.data[ind];
			++ind;
		}
	}
	return mat;
}

argus_msgs::SymmetricFloat64 SymmetricToMsg( const MatrixType& mat )
{
	if( mat.rows() != mat.cols() )
	{
		throw std::runtime_error( "SymmetricToMsg: Matrix not square." );
	}

	argus_msgs::SymmetricFloat64 msg;
	msg.dim = mat.rows();
	msg.data.reserve( msg.dim*(msg.dim+1)/2 );
	for( unsigned int i = 0; i < msg.dim; ++i )
	{
		for( unsigned int j = i; j < msg.dim; ++j )
		{
			msg.data.push_back( mat(i,j) );
		}
	}
	return msg;
}


}
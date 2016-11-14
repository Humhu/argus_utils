#pragma once

#include <Eigen/Dense>
#include <boost/array.hpp>
#include <cassert>

#include "argus_utils/ArgusTypes.h"
#include <argus_msgs/MatrixFloat64.h>
#include <argus_msgs/SymmetricFloat64.h>

namespace argus
{

enum MatrixStorageOrder
{
	RowMajor = 0,
	ColMajor
};

// TODO Move this into argus_msgs so we don't have a weird dependency!!
/*! \brief Conversion to and from the MatrixFloat64 message type. */
MatrixType MsgToMatrix( const argus_msgs::MatrixFloat64& msg );
argus_msgs::MatrixFloat64 MatrixToMsg( const MatrixType& mat );

// TODO Implement!
/*! \brief Conversion to and from the SymmetricFloat64 message type. 
 * Takes the lower triangle and assumes it is symmetric. */
MatrixType MsgToSymmetric( const argus_msgs::SymmetricFloat64& msg );
argus_msgs::SymmetricFloat64 SymmetricToMsg( const MatrixType& mat );

template <typename T>
Eigen::Map<const VectorType> GetVectorView( const T& mat )
{
	return Eigen::Map<const VectorType>( mat.data(), mat.size() );
}
template <typename T>
Eigen::Map<VectorType> GetVectorView( T& mat )
{
	return Eigen::Map<VectorType>( mat.data(), mat.size() );
}

// Generic parsing - requires that mat have set dimensions
template <typename Derived, typename Scalar>
bool ParseMatrix( const Scalar* src,
                  Eigen::DenseBase<Derived>& mat,
                  int order = RowMajor )
{
	unsigned int ind = 0;
	if( order == RowMajor )
	{
		for( unsigned int i = 0; i < mat.rows(); i++ )
		{
			for( unsigned int j = 0; j < mat.cols(); j++ )
			{
				mat(i,j) = src[ind];
				ind++;
			}
		}
	}
	else if( order == ColMajor )
	{
		for( unsigned int i = 0; i < mat.cols(); i++ )
		{
			for( unsigned int j = 0; j < mat.rows(); j++ )
			{
				mat(j,i) = src[ind];
				ind++;
			}
		}
	}
	else { return false; }
	return true;
}

// Parsing from a std vector
template <typename Derived, typename Scalar>
bool ParseMatrix( const std::vector<Scalar>& data,
                  Eigen::DenseBase<Derived>& mat,
                  int order = RowMajor )
{
	if( mat.rows()*mat.cols() != data.size() ) { return false; }
	
	return ParseMatrix<Derived,Scalar>( data.data(), mat, order );
}

// Parsing from a boost array
template <typename Derived, typename Scalar, unsigned long N>
bool ParseMatrix( const boost::array<Scalar,N>& data,
                  Eigen::DenseBase<Derived>& mat,
                  int order = RowMajor )
{
	if( mat.rows()*mat.cols() != N ) { return false; }
	
	return ParseMatrix<Derived,Scalar>( data.data(), mat, order );
}

// Parses an array as if it is the unique elements of a symmetric matrix. */
template <typename Derived, typename Scalar>
bool ParseSymmetricMatrix( const Scalar* src,
                           Eigen::DenseBase<Derived>& mat )
{
	if( mat.rows() != mat.cols() ) { return false; }
	
	unsigned int ind = 0;
	for( unsigned int i = 0; i < mat.rows(); i++ )
	{
		for( unsigned int j = i; j < mat.cols(); j++ )
		{
			mat(i,j) = src[ind];
			mat(j,i) = src[ind];
			ind++;
		}
	}
	return true;
}

template <typename Derived, typename Scalar>
bool ParseSymmetricMatrix( const std::vector<Scalar>& data,
                           Eigen::DenseBase<Derived>& mat )
{
	size_t n = mat.rows();
	if( ( (n+1) * n ) / 2 != data.size() ) { return false; }
	return ParseSymmetricMatrix( data.data(), mat );
}

template <typename Derived, typename Scalar, unsigned long N>
bool ParseSymmetricMatrix( const boost::array<Scalar,N>& data,
                           Eigen::DenseBase<Derived>& mat )
{
	size_t n = mat.rows();
	if( ( (n+1) * n ) / 2 != N ) { return false; }
	return ParseSymmetricMatrix( data.data(), mat );
}

// Generic serialization - avoid using unless necessary
template <typename Derived, typename Scalar>
bool SerializeMatrix( const Eigen::DenseBase<Derived>& mat, 
                      Scalar* dst,
                      MatrixStorageOrder order = ColMajor )
{
	unsigned int ind = 0;
	if( order == RowMajor )
	{
		for( unsigned int i = 0; i < mat.rows(); i++ )
		{
			for( unsigned int j = 0; j < mat.cols(); j++ )
			{
				dst[ind] = mat(i,j);
				ind++;
			}
		}
	}
	else if( order == ColMajor )
	{
		for( unsigned int i = 0; i < mat.cols(); i++ )
		{
			for( unsigned int j = 0; j < mat.rows(); j++ )
			{
				dst[ind] = mat(j,i);
				ind++;
			}
		}
	}
	else 
	{
		throw std::runtime_error( "SerializeMatrix: Invalid storage order." );
	}
	return true;
}

// General purpose std vectors
template <typename Derived, typename Scalar>
bool SerializeMatrix( const Eigen::DenseBase<Derived>& mat, 
                      std::vector<Scalar>& dst,
                      MatrixStorageOrder order = ColMajor )
{
	dst.resize( mat.rows()*mat.cols() );
	
	return SerializeMatrix<Derived, Scalar>( mat, dst.data(), order );
}

// Specific to ROS fixed-length array message fields (boost arrays)
template <typename Derived, typename Scalar, unsigned long N>
bool SerializeMatrix( const Eigen::DenseBase<Derived>& mat, 
                      boost::array<Scalar, N>& dst,
                      MatrixStorageOrder order = ColMajor )
{
	if( mat.rows()*mat.cols() != N ) { return false; }
	
	return SerializeMatrix<Derived, Scalar>( mat, dst.data(), order );
}

// Generic serialization for symmetric matrices. Stores in same order as parsing.
// Fails if matrix is not square or not symmetric
template <typename Derived, typename Scalar>
bool SerializeSymmetricMatrix( const Eigen::DenseBase<Derived>& mat,
                               Scalar* dst )
{
	if( mat.rows() != mat.cols() ) { return false; }
	
	unsigned int ind = 0;
	for( unsigned int i = 0; i < mat.rows(); i++ )
	{
		for( unsigned int j = i; j < mat.cols(); j++ )
		{
			dst[ind] = mat(i,j);
			ind++;
		}
	}
	return true;
}

template <typename Derived, typename Scalar>
bool SerializeSymmetricMatrix( const Eigen::DenseBase<Derived>& mat,
                      std::vector<Scalar>& dst )
{
	size_t n = mat.rows();
	dst.resize( ( (n+1) * n ) / 2 );
	return SerializeSymmetricMatrix<Derived, Scalar>( mat, dst.data() );
}

template <typename Derived, typename Scalar, unsigned long N>
bool SerializeSymmetricMatrix( const Eigen::DenseBase<Derived>& mat,
                      boost::array<Scalar, N>& dst )
{
	size_t n = mat.rows();
	if( ( (n+1) * n ) / 2 != N ) { return false; }
	return SerializeSymmetricMatrix<Derived, Scalar>( mat, dst.data() );
}

template <typename Derived>
void CheckIndices( const Eigen::DenseBase<Derived>& mat,
                   unsigned int i, unsigned int j )
{
	if( i > mat.rows() || j > mat.cols() )
	{
		std::stringstream ss;
		ss << "Attempted to access (" << i << ", " << j
		   << ") in a (" << mat.rows() << ", " << mat.cols() << ") matrix.";
		throw std::runtime_error( ss.str() );
	}
}

template <typename DerivedIn, typename DerivedOut, typename IndsType>
void GetSubmatrix( const Eigen::DenseBase<DerivedIn>& mat,
                   Eigen::DenseBase<DerivedOut>& sub,
                   const IndsType& rowInds,
                   const IndsType& colInds = {0} )
{
	unsigned int N = rowInds.size();
	unsigned int M = colInds.size();
	if( sub.rows() != N || sub.cols() != M )
	{
		throw std::invalid_argument( "Submatrix size must match indices size." );
	}
	
	for( unsigned int i = 0; i < N; i++ )
	{
		for( unsigned int j = 0; j < M; j++ )
		{
			CheckIndices( sub, i, j );
			CheckIndices( mat, rowInds[i], colInds[j] );
			sub(i,j) = mat( rowInds[i], colInds[j] );
		}
	}
}

template <typename DerivedIn, typename DerivedOut, typename IndsType>
void PutSubmatrix( Eigen::DenseBase<DerivedIn>& mat,
                   const Eigen::DenseBase<DerivedOut>& sub,
                   const IndsType& rowInds,
                   const IndsType& colInds )
{
	unsigned int N = rowInds.size();
	unsigned int M = colInds.size();
	if( sub.rows() != N || sub.cols() != M )
	{
		throw std::invalid_argument( "Submatrix invalid size." );
	}
	
	for( unsigned int i = 0; i < N; i++ )
	{
		for( unsigned int j = 0; j < N; j++ )
		{
			CheckIndices( sub, i, j );
			CheckIndices( mat, rowInds[i], colInds[j] );
			mat( rowInds[i], rowInds[j] ) = sub(i,j);
		}
	}
}

// Retrieves the lower triangular part of a matrix with optional offset
template <typename Derived>
VectorType GetLowerTriangular( const Eigen::DenseBase<Derived>& in,
                               unsigned int offset = 0 )
{
	if( in.rows() != in.cols() )
	{
		throw std::runtime_error( "GetLowerTriangular: Input must be square." );
	}
	unsigned int extents = in.cols();
	if( offset >= extents )
	{
		throw std::runtime_error( "GetLowerTriangular: Cannot have offset greater or equal to extents." );
	}
	unsigned int ind = 0;
	unsigned int dim = extents - offset;
	VectorType out( dim*(dim+1)/2 );
	for( unsigned int j = 0; j < extents - offset; ++j )
		{
		for( unsigned int i = j+offset; i < extents; ++i )
		{
			out(ind++) = in(i,j);
		}
	}
	return out;
}

} // end namespace argus

#pragma once

#include <Eigen/Dense>
#include <boost/array.hpp>
#include <cassert>

namespace argus_utils
{

enum MatrixStorageOrder
{
	RowMajor = 0,
	ColMajor
};

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

// Generic serialization - avoid using unless necessary
template <typename Derived, typename Scalar>
bool SerializeMatrix( const Eigen::DenseBase<Derived>& mat, 
					  Scalar* dst,
					  int order = RowMajor )
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
	else { return false; }
	return true;
}

// General purpose std vectors
template <typename Derived, typename Scalar>
bool SerializeMatrix( const Eigen::DenseBase<Derived>& mat, 
					  std::vector<Scalar>& dst,
					  int order = RowMajor )
{
	dst.resize( mat.rows()*mat.cols() );
	
	return SerializeMatrix<Derived, Scalar>( mat, dst.data(), order );
}

// Specific to ROS fixed-length array message fields (boost arrays)
template <typename Derived, typename Scalar, unsigned long N>
bool SerializeMatrix( const Eigen::DenseBase<Derived>& mat, 
					  boost::array<Scalar, N>& dst,
					  int order = RowMajor )
{
	if( mat.rows()*mat.cols() != N ) { return false; }
	
	return SerializeMatrix<Derived, Scalar>( mat, dst.data(), order );
}

template <typename DerivedIn, typename DerivedOut, unsigned long N, unsigned long M>
bool GetSubmatrix( const Eigen::DenseBase<DerivedIn>& mat,
				   Eigen::DenseBase<DerivedOut>& sub,
				   const std::array<unsigned int, N>& rowInds,
				   const std::array<unsigned int, M>& colInds )
{
	assert( sub.rows()*sub.cols() == N*M );
	
	for( unsigned int i = 0; i < N; i++ )
	{
		assert( rowInds[i] < mat.rows() );
	}
	for( unsigned int i = 0; i < M; i++ )
	{
		assert( colInds[i] < mat.cols() );
	}
	
	for( unsigned int i = 0; i < N; i++ )
	{
		for( unsigned int j = 0; j < N; j++ )
		{
			sub(i,j) = mat( rowInds[i], rowInds[j] );
		}
	}
	
	return true; // TODO Get rid of these useless bools
}

template <typename DerivedIn, typename DerivedOut, unsigned long N, unsigned long M>
bool PutSubmatrix( Eigen::DenseBase<DerivedIn>& mat,
				   const Eigen::DenseBase<DerivedOut>& sub,
				   const std::array<unsigned int, N>& rowInds,
				   const std::array<unsigned int, M>& colInds )
{
	assert( sub.rows()*sub.cols() == N*M );
	
	for( unsigned int i = 0; i < N; i++ )
	{
		assert( rowInds[i] < mat.rows() );
	}
	for( unsigned int i = 0; i < M; i++ )
	{
		assert( colInds[i] < mat.cols() );
	}
	
	for( unsigned int i = 0; i < N; i++ )
	{
		for( unsigned int j = 0; j < N; j++ )
		{
			mat( rowInds[i], rowInds[j] ) = sub(i,j);
		}
	}
	
	return true; // TODO Get rid of these useless bools
}

} // end namespace argus_utils
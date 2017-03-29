#pragma once

#include <Eigen/Dense>

namespace argus
{

// Linear algebra standardization typedefs
// All typedefs use double precision
// Matrix defaults are column-major
// Vector defaults are column-oriented

// Dynamic-sized matrix typedefs
typedef Eigen::Matrix <double, 
                       Eigen::Dynamic, 
                       Eigen::Dynamic, 
                       Eigen::ColMajor> ColMatrixType;
typedef Eigen::Matrix <double, 
                       Eigen::Dynamic, 
                       Eigen::Dynamic, 
                       Eigen::RowMajor> RowMatrixType;
typedef ColMatrixType MatrixType;

// Dynamic-sized vector typedefs
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ColVectorType;
typedef Eigen::Matrix<double, 1, Eigen::Dynamic> RowVectorType;
typedef ColVectorType VectorType;

// Fixed-size matrix typedefs
template <int Rows, int Cols>
using FixedColMatrixType = Eigen::Matrix <double, Rows, Cols, 
                                          Eigen::ColMajor>;
template <int Rows, int Cols>
using FixedRowMatrixType = Eigen::Matrix <double, Rows, Cols, 
                                          Eigen::RowMajor>;
template <int Rows, int Cols>
using FixedMatrixType = FixedColMatrixType<Rows, Cols>;

// Fixed-size vector typedefs
template <int Dim>
using FixedColVectorType = FixedMatrixType<Dim,1>;
template <int Dim>
using FixedRowVectorType = FixedMatrixType<1,Dim>;
template <int Dim>
using FixedVectorType = FixedColVectorType<Dim>;

// Partial fixed matrix typedefs
template <int Rows>
using HeightFixedColMatrixType = Eigen::Matrix <double, Rows,
                                            Eigen::Dynamic,
                                            Eigen::ColMajor>;
template <int Rows>
using HeightFixedRowMatrixType = Eigen::Matrix <double, Rows,
                                            Eigen::Dynamic,
                                            Eigen::RowMajor>;
template <int Rows>
using HeightFixedMatrixType = HeightFixedColMatrixType<Rows>;

template <int Cols>
using WidthFixedColMatrixType = Eigen::Matrix <double, Eigen::Dynamic,
                                           Cols, Eigen::ColMajor>;
template <int Cols>
using WidthFixedRowMatrixType = Eigen::Matrix <double, Eigen::Dynamic,
                                           Cols, Eigen::RowMajor>;
template <int Cols>
using WidthFixedMatrixType = WidthFixedColMatrixType<Cols>;

typedef Eigen::Map<MatrixType> MatrixViewType;
typedef Eigen::Map<const MatrixType> ConstMatrixViewType;
typedef Eigen::Map<VectorType> VectorViewType;
typedef Eigen::Map<const VectorType> ConstVectorViewType;

}
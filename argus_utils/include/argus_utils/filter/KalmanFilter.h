#pragma once

#include "argus_utils/utils/LinalgTypes.h"
#include "argus_utils/filter/FilterInfo.h"

namespace argus
{
class KalmanFilter
{
public:

	KalmanFilter();
	void Initialize( const VectorType& x, const MatrixType& P );

	// NOTE Optional
	void SetTransitionMatrix( const MatrixType& A );
	void SetTransitionCovariance( const MatrixType& Q );
	void SetObservationMatrix( const MatrixType& C );
	void SetObservationCovariance( const MatrixType& R );

	PredictInfo Predict();
	PredictInfo Predict( const MatrixType& A, const MatrixType& Q );
	PredictInfo Predict( const VectorType& dx );
	PredictInfo Predict( const VectorType& dx, const MatrixType& A, const MatrixType& Q );

	UpdateInfo Update( const VectorType& y );
	UpdateInfo Update( const VectorType& y, const MatrixType& C, const MatrixType& R );

	const VectorType& GetState() const;
	const MatrixType& GetCovariance() const;

private:

	VectorType _x;
	MatrixType _P;

	MatrixType _A;
	MatrixType _Q;
	MatrixType _C;
	MatrixType _R;
};
}
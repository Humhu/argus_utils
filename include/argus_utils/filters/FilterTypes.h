#pragma once

#include "argus_utils/filters/PoseKalmanFilter.hpp"
#include "argus_utils/filters/DerivativePoseFilter.hpp"

#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{

typedef PoseKalmanFilter<PoseSE2> PoseFilterSE2;
typedef PoseKalmanFilter<PoseSE2> PoseFilterSE3;

template <typename P>
using ConstantVelocityPoseFilter = DerivativePoseFilter<P, 1>;
template <typename P>
using ConstantAccelPoseFilter = DerivativePoseFilter<P, 2>;
template <typename P>
using ConstantJerkPoseFilter = DerivativePoseFilter<P, 3>;

typedef ConstantVelocityPoseFilter<PoseSE2> ConstantVelocityFilterSE2;
typedef ConstantAccelPoseFilter<PoseSE2> ConstantAccelFilterSE2;
typedef ConstantJerkPoseFilter<PoseSE2> ConstantJerkFilterSE2;

typedef ConstantVelocityPoseFilter<PoseSE3> ConstantVelocityFilterSE3;
typedef ConstantAccelPoseFilter<PoseSE3> ConstantAccelFilterSE3;
typedef ConstantJerkPoseFilter<PoseSE3> ConstantJerkFilterSE3;

}
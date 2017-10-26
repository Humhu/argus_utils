#pragma once

#include <map>

#include "argus_utils/synchronization/SynchronizationTypes.h"
#include "argus_utils/utils/MapUtils.hpp"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{
template <typename Pose>
class VelocityIntegrator
{
public:

	typedef Pose PoseType;
	typedef typename Pose::TangentVector VelocityType;
	typedef typename Pose::CovarianceMatrix CovarianceType;

	VelocityIntegrator() : _maxBuffLen( 5.0 ) {}

	VelocityIntegrator( const VelocityIntegrator& other )
	: _maxBuffLen( other._maxBuffLen ),
	  _buffer( other._buffer )
	{}

	void Reset()
	{
		WriteLock lock( _mutex );
		_buffer.clear();
	}

	void SetMaxBuffLen( double dt )
	{
		WriteLock lock( _mutex );
		_maxBuffLen = dt;
	}

	void BufferInfo( double time,
	                 const VelocityType& vel,
	                 const CovarianceType& cov )
	{
		WriteLock lock( _mutex );
		_buffer[time] = TangentInfo( vel, cov );

		while( !_buffer.empty() )
		{
			double latest = get_highest_key(_buffer);
			double earliest = get_lowest_key(_buffer); 
			if( (latest - earliest) > _maxBuffLen )
			{
				remove_lowest( _buffer );
			}
			else { break; }
		}
	}

	VelocityType GetLatestVelocity() const
	{
		ReadLock lock( _mutex );
		VelocityType vel = VelocityType::Zero();
		if( _buffer.size() > 0 )
		{
			double latest = get_highest_key(_buffer);
			vel = _buffer.at(latest).first;
		}
		return vel;
	}

	CovarianceType GetLatestCovariance() const
	{
		ReadLock lock( _mutex );
		CovarianceType cov = CovarianceType::Zero();
		if( _buffer.size() > 0 )
		{
			double latest = get_highest_key(_buffer);
			cov = _buffer.at(latest).second;
		}
		return cov;
	}

	bool Integrate( double start, double finish,
	                PoseType& disp,
	                CovarianceType& cov,
					bool extrapolate = true ) const
	{
		WriteLock lock( _mutex );
		TangentBuffer::const_iterator iter;
		if( !get_closest_lesser_eq( _buffer, start, iter ) ) { return false; }

		disp = PoseSE3();
		cov = CovarianceType::Zero();
		
		double prevTime = start;
		TangentInfo prevInfo = iter->second;
		bool done = false;
		while( ++iter != _buffer.end() && !done )
		{
			double currTime = iter->first;
			TangentInfo currInfo = iter->second;

			if( currTime > finish )
			{
				currTime = finish;
				done = true;
			}

			double dt = currTime - prevTime;
			// TODO Linear interpolation instead of ZOH?
			disp = disp * PoseType::Exp( prevInfo.first * dt );
			cov += prevInfo.second * dt;

			prevTime = currTime;
			prevInfo = currInfo;
		}

		if( done ) { return true; }
		
		if( extrapolate )
		{
			double dt = finish - prevTime;
			disp = disp * PoseType::Exp( prevInfo.first * dt );
			cov += prevInfo.second * dt;
			return true;
		}

		return false;
	}

private:

	double _maxBuffLen;

	// NOTE We avoid fixed types due to alignment pains
	typedef std::pair<VectorType, MatrixType> TangentInfo;
	typedef std::map<double, TangentInfo> TangentBuffer;

	mutable Mutex _mutex;
	TangentBuffer _buffer;
};

typedef VelocityIntegrator<PoseSE2> VelocityIntegratorSE2;
typedef VelocityIntegrator<PoseSE3> VelocityIntegratorSE3;

}
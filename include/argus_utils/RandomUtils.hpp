#pragma once

#include <boost/random/random_number_generator.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace argus_utils
{

	
/*! \brief Boost-based reservoir sampling implementation. Best used when sampling
 * a medium subset. */
template<class Engine>
void ReservoirSampling( unsigned int numItems, unsigned int subsetSize,
						std::vector<unsigned int>& inds, Engine& engine )
{
	inds.resize( subsetSize );
	for( unsigned int i = 0; i < subsetSize; i++ )
	{
		inds[i] = i;
	}
	
	for( unsigned int i = subsetSize; i < numItems; i++ )
	{
		boost::random::uniform_int_distribution<> dist( 0, i ); // i inclusive
		unsigned int j = (unsigned int) dist( engine );
		std::cout << "j: " << j << " ";
		if( j < subsetSize )
		{
			inds[j] = i;
		}
	}
	std::cout << std::endl;
}
	
/*! \brief Boost-based brute force sampling implementation. Best used when
 * sampling a small subset. */
template<class Engine>
void BitmapSampling( unsigned int numItems, unsigned int subsetSize,
					 std::vector<unsigned int>& inds, Engine& engine )
{
	inds.resize( subsetSize );
	
	std::vector<bool> bitmap( numItems, false );
	unsigned int count = 0;
	boost::random::uniform_int_distribution<> dist( 0, numItems - 1 );
	while( count < subsetSize )
	{
		unsigned int j = (unsigned int) dist( engine );
		if( bitmap[j] ) { continue; }
		bitmap[j] = true;
		inds[count] = j;
		count++;
	}
	
}

}

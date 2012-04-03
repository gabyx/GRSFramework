
#ifndef SharedBufferPlayback_hpp
#define SharedBufferPlayback_hpp

#include <Eigen/Dense>
#include <boost/thread.hpp>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "StateRingPoolVisBackFront.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the SharedBufferPlayback class which is a specialisation which is used to add several more shared data to the StateRingPoolVisBackFront base class.
*/
template< typename TLayoutConfig>
class SharedBufferPlayback : public StateRingPoolVisBackFront<TLayoutConfig>
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//Static and Dynamic allocation of the matrices in the class
	SharedBufferPlayback(unsigned int nSimBodies);
	~SharedBufferPlayback();

	void	reset();

private:

};


   using namespace std;
template< typename TLayoutConfig>
SharedBufferPlayback<TLayoutConfig>::SharedBufferPlayback(unsigned int nSimBodies):
SharedBufferPlayback<TLayoutConfig>::StateRingPoolVisBackFront(nSimBodies)
{
	reset();
}

template< typename TLayoutConfig>
SharedBufferPlayback<TLayoutConfig>::~SharedBufferPlayback(){
  DECONSTRUCTOR_MESSAGE
}

template< typename TLayoutConfig>
void SharedBufferPlayback<TLayoutConfig>::reset(){
	StateRingPoolVisBackFront<TLayoutConfig>::resetStateRingPool();
}


#endif

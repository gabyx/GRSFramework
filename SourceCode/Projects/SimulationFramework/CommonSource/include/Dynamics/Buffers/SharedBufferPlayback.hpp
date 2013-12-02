
#ifndef SharedBufferPlayback_hpp
#define SharedBufferPlayback_hpp

#include <boost/thread.hpp>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "StateRingPoolVisBackFront.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the SharedBufferPlayback class which is a specialisation which is used to add several more shared data to the StateRingPoolVisBackFront base class.
*/

class SharedBufferPlayback : public StateRingPoolVisBackFront
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//Static and Dynamic allocation of the matrices in the class
	SharedBufferPlayback(unsigned int nSimBodies);
	~SharedBufferPlayback();

	void	reset();

private:

};



SharedBufferPlayback::SharedBufferPlayback(unsigned int nSimBodies):
StateRingPoolVisBackFront(nSimBodies)
{
	reset();
}


SharedBufferPlayback::~SharedBufferPlayback(){
  DECONSTRUCTOR_MESSAGE
}


void SharedBufferPlayback::reset(){
	StateRingPoolVisBackFront::resetStateRingPool();
}


#endif

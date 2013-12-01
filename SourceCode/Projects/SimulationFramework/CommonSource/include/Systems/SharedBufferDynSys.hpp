
#ifndef SharedBufferDynSys_hpp
#define SharedBufferDynSys_hpp

#include <boost/thread.hpp>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "StatePoolVisBackFront.hpp"


class SharedBufferDynSys : public StatePoolVisBackFront
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//Static and Dynamic allocation of the matrices in the class
	SharedBufferDynSys(unsigned int nSimBodies);

	~SharedBufferDynSys();

	void	reset();

private:



};

SharedBufferDynSys::SharedBufferDynSys(unsigned int nSimBodies):
StatePoolVisBackFront(nSimBodies)
{
	reset();
}


SharedBufferDynSys::~SharedBufferDynSys(){
  DECONSTRUCTOR_MESSAGE
}


void SharedBufferDynSys::reset(){
	this->resetStatePool();
}



#endif

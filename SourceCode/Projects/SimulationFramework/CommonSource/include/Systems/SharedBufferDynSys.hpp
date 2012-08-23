
#ifndef SharedBufferDynSys_hpp
#define SharedBufferDynSys_hpp

#include <Eigen/Dense>
#include <boost/thread.hpp>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "StatePoolVisBackFront.hpp"


template< typename TLayoutConfig>
class SharedBufferDynSys : public StatePoolVisBackFront<TLayoutConfig>
{
public:
   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	//Static and Dynamic allocation of the matrices in the class
	SharedBufferDynSys(unsigned int nSimBodies);

	~SharedBufferDynSys();

	void	reset();

private:



};
template< typename TLayoutConfig>
SharedBufferDynSys<TLayoutConfig>::SharedBufferDynSys(unsigned int nSimBodies):
StatePoolVisBackFront<TLayoutConfig>(nSimBodies)
{
	reset();
}

template< typename TLayoutConfig>
SharedBufferDynSys<TLayoutConfig>::~SharedBufferDynSys(){
  DECONSTRUCTOR_MESSAGE
}

template< typename TLayoutConfig>
void SharedBufferDynSys<TLayoutConfig>::reset(){
	this->resetStatePool();
}



#endif

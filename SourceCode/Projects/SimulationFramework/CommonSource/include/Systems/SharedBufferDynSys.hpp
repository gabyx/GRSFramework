
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

	template<typename RigidBodyIterator>
    SharedBufferDynSys(RigidBodyIterator itBegin, RigidBodyIterator itEnd):
        StatePoolVisBackFront(itBegin,itEnd)
    {};

	~SharedBufferDynSys(){
      DECONSTRUCTOR_MESSAGE
    };

private:



};


#endif


#ifndef GRSF_Systems_SharedBufferDynSys_hpp
#define GRSF_Systems_SharedBufferDynSys_hpp

#include <boost/thread.hpp>
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/Buffers/StatePoolVisBackFront.hpp"


class SharedBufferDynSys : public StatePoolVisBackFront
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

	template<typename TRigidBodyIterator>
    SharedBufferDynSys(TRigidBodyIterator beg, TRigidBodyIterator end):
        StatePoolVisBackFront(beg,end)
    {};

	~SharedBufferDynSys(){
      DECONSTRUCTOR_MESSAGE
    };

private:



};


#endif

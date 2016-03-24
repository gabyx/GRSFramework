
#ifndef GRSF_systems_SharedBufferDynSys_hpp
#define GRSF_systems_SharedBufferDynSys_hpp

#include <boost/thread.hpp>
#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/buffers/StatePoolVisBackFront.hpp"


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

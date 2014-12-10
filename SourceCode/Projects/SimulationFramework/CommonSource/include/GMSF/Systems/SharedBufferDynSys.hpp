
#ifndef GMSF_Systems_SharedBufferDynSys_hpp
#define GMSF_Systems_SharedBufferDynSys_hpp

#include <boost/thread.hpp>
#include "GMSF/Common/AssertionDebug.hpp"

#include "GMSF/Common/TypeDefs.hpp"

#include "GMSF/Dynamics/Buffers/StatePoolVisBackFront.hpp"


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

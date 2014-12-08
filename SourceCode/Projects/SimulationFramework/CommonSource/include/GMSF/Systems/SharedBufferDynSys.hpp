
#ifndef GMSF_Systems_SharedBufferDynSys_hpp
#define GMSF_Systems_SharedBufferDynSys_hpp

#include <boost/thread.hpp>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "StatePoolVisBackFront.hpp"


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

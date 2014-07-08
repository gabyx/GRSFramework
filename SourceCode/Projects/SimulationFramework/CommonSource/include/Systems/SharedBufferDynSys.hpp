
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

	template<typename TRigidBodyCont>
    SharedBufferDynSys(const TRigidBodyCont & cont):
        StatePoolVisBackFront(cont)
    {};

	~SharedBufferDynSys(){
      DECONSTRUCTOR_MESSAGE
    };

private:



};


#endif

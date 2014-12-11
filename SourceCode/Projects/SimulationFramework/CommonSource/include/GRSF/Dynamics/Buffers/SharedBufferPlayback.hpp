#ifndef GRSF_Dynamics_Buffers_SharedBufferPlayback_hpp
#define GRSF_Dynamics_Buffers_SharedBufferPlayback_hpp

#include <boost/thread.hpp>
#include "GRSF/Common/AssertionDebug.hpp"

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the SharedBufferPlayback class which is a specialisation which is used to add several more shared data to the StateRingPoolVisBackFront base class.
*/

class SharedBufferPlayback : public StateRingPoolVisBackFront {
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    template<typename RigidBodyIterator>
    SharedBufferPlayback(RigidBodyIterator itBegin, RigidBodyIterator itEnd):
        StateRingPoolVisBackFront(itBegin,itEnd)
    {};

    ~SharedBufferPlayback() {
        DECONSTRUCTOR_MESSAGE
    };

private:

};

#endif

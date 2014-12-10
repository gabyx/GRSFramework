#ifndef GMSF_Dynamics_Buffers_SharedBufferPlayback_hpp
#define GMSF_Dynamics_Buffers_SharedBufferPlayback_hpp

#include <boost/thread.hpp>
#include "GMSF/Common/AssertionDebug.hpp"

#include "GMSF/Common/TypeDefs.hpp"
#include "GMSF/Common/LogDefines.hpp"

#include "GMSF/Dynamics/Buffers/StateRingPoolVisBackFront.hpp"


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

#ifndef GRSF_dynamics_buffers_SharedBufferPlayback_hpp
#define GRSF_dynamics_buffers_SharedBufferPlayback_hpp

#include <boost/thread.hpp>
#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include "GRSF/dynamics/buffers/StateRingPoolVisBackFront.hpp"


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

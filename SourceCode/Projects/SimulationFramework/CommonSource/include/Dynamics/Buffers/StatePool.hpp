#ifndef StatePool_hpp
#define StatePool_hpp

#include <iostream>
#include <fstream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Core>


#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "DynamicsState.hpp"

#define DECLERATIONS_STATEPOOL \
    using StatePool::m_change_pointer_mutex; \
    using StatePool::m_idx; \
    using StatePool::m_pool; \
    typedef typename StatePool::atomic_char atomic_char;

/**
* @ingroup StatesAndBuffers
* @brief This is the StatePool class which is a general base class for different Pools, e.g RingPool etc.
* @{
*/

class StatePool {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StatePool(const unsigned int nIndices) {
        // Allocate how many pointers we have!
        m_idx = new atomic_char[nIndices];
    }
    ~StatePool(){
        DECONSTRUCTOR_MESSAGE
        // Allocate how many pointers we have!
        delete[] m_idx;
    };

protected:

    boost::mutex    m_change_pointer_mutex; ///< This is the mutex which is used to have a mutual exclusion if the pointers on the buffer changes.
    std::vector<boost::shared_ptr<DynamicsState > >  m_pool; ///< This is the vector of states which are present in the pool. The subclass implement how many such states are in the pool.
    typedef volatile unsigned char atomic_char;
    atomic_char*   m_idx; ///< These are the indices into the pool m_pool. The subclasses handle this indices.

};
/** @} */



#endif

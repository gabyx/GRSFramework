// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_StatePool_hpp
#define GRSF_dynamics_buffers_StatePool_hpp

#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <boost/thread.hpp>
#include <Eigen/Core>


#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#define DECLERATIONS_STATEPOOL \
    using StatePool::m_change_pointer_mutex; \
    using StatePool::m_idx; \
    using StatePool::m_pool; \
    using atomic_char = typename StatePool::atomic_char;

/**
* @ingroup StatesAndBuffers
* @brief This is the StatePool class which is a general base class for different Pools, e.g RingPool etc.
* @{
*/
template<typename StateType>
class StatePool {
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    StatePool(const unsigned int nIndices) {
        // Allocate how many pointers we have!
        m_idx = new atomic_char[nIndices];
    }
    ~StatePool(){
        DESTRUCTOR_MESSAGE
        // Allocate how many pointers we have!
        delete[] m_idx;
    };

protected:

    boost::mutex    m_change_pointer_mutex; ///< This is the mutex which is used to have a mutual exclusion if the pointers on the buffer changes.
    std::vector<StateType>  m_pool; ///< This is the vector of states which are present in the pool. The subclass implement how many such states are in the pool.
    using atomic_char = volatile unsigned char;
    atomic_char*   m_idx; ///< These are the indices into the pool m_pool. The subclasses handle this indices.

};
/** @} */



#endif

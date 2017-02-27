// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_buffers_SharedBufferPlayback_hpp
#define GRSF_dynamics_buffers_SharedBufferPlayback_hpp

#include <boost/thread.hpp>
#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/buffers/StateRingPoolVisBackFront.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the SharedBufferPlayback class which is a specialisation which is used to add several more shared data
* to the StateRingPoolVisBackFront base class.
*/

class SharedBufferPlayback : public StateRingPoolVisBackFront
{
    public:
    DEFINE_LAYOUT_CONFIG_TYPES

    template <typename RigidBodyIterator>
    SharedBufferPlayback(RigidBodyIterator itBegin, RigidBodyIterator itEnd)
        : StateRingPoolVisBackFront(itBegin, itEnd){};

    ~SharedBufferPlayback(){DESTRUCTOR_MESSAGE};

    private:
};

#endif

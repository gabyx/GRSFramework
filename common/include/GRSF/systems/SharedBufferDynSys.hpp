// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_systems_SharedBufferDynSys_hpp
#define GRSF_systems_SharedBufferDynSys_hpp

#include <boost/thread.hpp>
#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/buffers/StatePoolVisBackFront.hpp"

class SharedBufferDynSys : public StatePoolVisBackFront
{
    public:
    DEFINE_LAYOUT_CONFIG_TYPES

    template <typename TRigidBodyIterator>
    SharedBufferDynSys(TRigidBodyIterator beg, TRigidBodyIterator end) : StatePoolVisBackFront(beg, end){};

    ~SharedBufferDynSys(){DESTRUCTOR_MESSAGE};

    private:
};

#endif

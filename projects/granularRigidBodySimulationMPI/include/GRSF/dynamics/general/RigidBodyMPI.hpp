// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodyMPI_hpp
#define GRSF_dynamics_general_RigidBodyMPI_hpp

#include "GRSF/dynamics/general/RigidBody.hpp"

#include "GRSF/dynamics/general/BodyInfoMap.hpp"

class RigidBodyBaseMPI : public RigidBodyBase
{
    public:
    using AbsoluteBaseType =
        RigidBodyBase;  ///< The absolut base type where m_id is defined, for the rigid body container
    using BodyInfoType = BodyProcessInfo;

    RigidBodyBaseMPI(const RigidBodyIdType& id) : RigidBodyBase(id), m_pBodyInfo(nullptr){};
    ~RigidBodyBaseMPI()
    {
        if (m_pBodyInfo)
        {
            delete m_pBodyInfo;
            m_pBodyInfo = nullptr;
        }
    };

    /**
    This is a class which contains all related info for the mpi information,
    only local and remote bodies have such a type, these get assigned during body communication and in the
    BodyCommunicator constructor
    */
    BodyInfoType* m_pBodyInfo;
};

#endif

// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodyId_hpp
#define GRSF_dynamics_general_RigidBodyId_hpp

#include <sstream>
#include <string>

namespace RigidBodyId
{
using Type     = uint64_t;  ///< [ --- last 32bit is GroupNr --- , --- first 32bit is BodyNr --- ]
using HalfType = uint32_t;

template <typename TRigidBodyType>
inline static HalfType getGroupNr(const TRigidBodyType* body)
{
    return static_cast<HalfType>(body->m_id >> sizeof(HalfType) * 8);
};

template <typename TRigidBodyType>
inline static HalfType getBodyNr(const TRigidBodyType* body)
{
    return static_cast<HalfType>(body->m_id);
};

inline static HalfType getGroupNr(const Type& id)
{
    return static_cast<HalfType>(id >> sizeof(HalfType) * 8);
};

inline static HalfType getBodyNr(const Type& id)
{
    return static_cast<HalfType>(id);
};

template <typename TRigidBodyType>
inline static std::string getBodyIdString(const TRigidBodyType* body)
{
    std::stringstream s;
    s << "(" << RigidBodyId::getGroupNr(body) << "," << RigidBodyId::getBodyNr(body) << ")";
    return s.str();
};

inline static std::string getBodyIdString(const Type& id)
{
    std::stringstream s;
    s << "(" << RigidBodyId::getGroupNr(id) << "," << RigidBodyId::getBodyNr(id) << ")";
    return s.str();
};

inline static Type makeId(HalfType groupNr, HalfType bodyNr)
{
    Type res = 0;
    res |= (uint64_t)groupNr;
    res <<= 32;
    res |= (uint64_t)bodyNr;
    return res;
};
};

/** Definition of the RigidBodyId type */
using RigidBodyIdType = RigidBodyId::Type;

/** Definition of the RigidBodyId half type */
using RigidBodyIdHalfType = RigidBodyId::HalfType;

#endif

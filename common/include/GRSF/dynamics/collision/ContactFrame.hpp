// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_collision_ContactFrame_hpp
#define GRSF_dynamics_collision_ContactFrame_hpp

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

class ContactFrame
{
public:
    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactFrame()
    {
        m_e_x.setZero();
        m_e_y.setZero();
        m_e_z.setZero();
        m_p.setZero();
    }
    Vector3 m_e_x;  /// e_x in frame I
    Vector3 m_e_y;  /// e_y in frame I
    Vector3 m_e_z;  /// e_z in frame I
    Vector3 m_p;    /// location of the contact in frame I
};

#endif

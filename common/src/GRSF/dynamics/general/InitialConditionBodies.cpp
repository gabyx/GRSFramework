// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/dynamics/general/InitialConditionBodies.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/dynamics/general/QuaternionHelpers.hpp"


//void InitialConditionBodies::setupPositionBodyPosAxisAngle(RigidBodyState & rigibodyState,
//                                                           const typename RigidBodyState::Vector3 & pos,
//                                                           typename RigidBodyState::Vector3 & axis,
//                                                           typename RigidBodyState::PREC angleRadian) {
//
//    rigibodyState.m_q.head<3>() = pos;
//    rigibodyState.m_q.tail<4>() = Quaternion(AngleAxis(angleRadian,axis)).coeffs();
//}


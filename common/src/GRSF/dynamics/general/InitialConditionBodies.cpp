
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


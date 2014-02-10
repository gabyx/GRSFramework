
#include "TypeDefs.hpp"
#include "InitialConditionBodies.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"



void InitialConditionBodies::setupPositionBodyPosAxisAngle(RigidBodyState & rigibodyState,
                                                           const typename RigidBodyState::Vector3 & pos,
                                                           typename RigidBodyState::Vector3 & axis,
                                                           typename RigidBodyState::PREC angleRadian) {

    rigibodyState.m_q.head<3>() = pos;
    setQuaternion(rigibodyState.m_q.tail<4>(),axis,angleRadian);
}


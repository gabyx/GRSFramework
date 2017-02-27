// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodyFunctions_hpp
#define GRSF_dynamics_general_RigidBodyFunctions_hpp

namespace RigidBodyFunctions
{
template <typename TRigidBody>
void initMassMatrixInv(TRigidBody* body)
{
    // Massmatrix Inverse
    body->m_MassMatrixInv_diag = body->m_MassMatrix_diag.array().inverse().matrix();
}

template <typename TRigidBody>
void initMassMatrix(TRigidBody* body)
{
    // Mass Matrix
    body->m_MassMatrix_diag.template head<3>().setConstant(body->m_mass);
    body->m_MassMatrix_diag.template tail<3>() = body->m_K_Theta_S;

    // Massmatrix Inverse
    initMassMatrixInv<TRigidBody>(body);
}
};

#endif  // RigidBodyFunctions_hpp

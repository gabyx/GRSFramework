// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_RigidBodyFunctionsMPI_hpp
#define GRSF_dynamics_general_RigidBodyFunctionsMPI_hpp


#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/dynamics/general/RigidBodyFunctions.hpp"

namespace RigidBodyFunctions {


    template<typename TRigidBody>
    void remoteToLocalBodyInitialization(TRigidBody * body){
         GRSF_ASSERTMSG( body->m_pSolverData, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << "no solver data!");
         body->m_pSolverData->reset();
    }

    template<typename TRigidBody>
    void changeBodyToSplitWeighting(TRigidBody * body,
                             const unsigned int & multiplicity,
                             const typename TRigidBody::PREC & multiplicityWeight){

            GRSF_ASSERTMSG( body->m_pSolverData, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << "no solver data!");
            GRSF_ASSERTMSG( body->m_pSolverData->m_multiplicity == 1, "Body id: "
                      << RigidBodyId::getBodyIdString(body->m_id) << " has multiplcity : "
                      << body->m_pSolverData->m_multiplicity)

            body->m_pSolverData->m_multiplicityWeight = multiplicityWeight;
            body->m_pSolverData->m_multiplicity = multiplicity;

            body->m_h_term *= multiplicityWeight; // h * alpha_i = h_i
            body->m_MassMatrixInv_diag *= 1.0/multiplicityWeight; // M-¹ * 1/alpha_i = M-¹_i

    };

    template<typename TRigidBody>
    void changeBodyToNormalWeighting(TRigidBody * body){
            GRSF_ASSERTMSG( body->m_pSolverData, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << "no solver data!");
            GRSF_ASSERTMSG( body->m_pSolverData->m_multiplicity != 1, "Body id: "
                      << RigidBodyId::getBodyIdString(body->m_id) << " has multiplicity : "
                      << body->m_pSolverData->m_multiplicity)

            body->m_h_term *= 1.0/body->m_pSolverData->m_multiplicityWeight;  // h_i * 1/alpha_i = h

            body->m_pSolverData->m_multiplicity = 1;
            body->m_pSolverData->m_multiplicityWeight = 1.0;


            RigidBodyFunctions::initMassMatrixInv<TRigidBody>(body);
    };

};

#endif // RigidBodyFunctionsMPI_hpp

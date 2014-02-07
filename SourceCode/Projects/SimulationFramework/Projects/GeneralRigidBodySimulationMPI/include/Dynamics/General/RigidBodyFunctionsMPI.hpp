#ifndef RigidBodyFunctionsMPI_hpp
#define RigidBodyFunctionsMPI_hpp


#include "AssertionDebug.hpp"

#include "RigidBodyFunctions.hpp"

namespace RigidBodyFunctions {


    template<typename TRigidBody>
    void remoteToLocalBodyInitialization(TRigidBody * body){
         ASSERTMSG( body->m_pSolverData, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << "no solver data!");
         body->m_pSolverData->reset();
    }

    template<typename TRigidBody>
    void changeBodyToSplitWeighting(TRigidBody * body,
                             const unsigned int & multiplicity,
                             const typename TRigidBody::PREC & multiplicityWeight){

            ASSERTMSG( body->m_pSolverData, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << "no solver data!");
            ASSERTMSG( body->m_pSolverData->m_multiplicity == 1, "Body id: "
                      << RigidBodyId::getBodyIdString(body->m_id) << " has multiplcity : "
                      << body->m_pSolverData->m_multiplicity)

            body->m_pSolverData->m_multiplicityWeight = multiplicityWeight;
            body->m_pSolverData->m_multiplicity = multiplicity;

            body->m_h_term *= multiplicityWeight; // h * alpha_i = h_i
            body->m_MassMatrixInv_diag *= 1.0/multiplicityWeight; // M-¹ * 1/alpha_i = M-¹_i

    };

    template<typename TRigidBody>
    void changeBodyToNormalWeighting(TRigidBody * body){
            ASSERTMSG( body->m_pSolverData, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << "no solver data!");
            ASSERTMSG( body->m_pSolverData->m_multiplicity != 1, "Body id: "
                      << RigidBodyId::getBodyIdString(body->m_id) << " has multiplicity : "
                      << body->m_pSolverData->m_multiplicity)

            body->m_h_term *= 1.0/body->m_pSolverData->m_multiplicityWeight;  // h_i * 1/alpha_i = h

            body->m_pSolverData->m_multiplicity = 1;
            body->m_pSolverData->m_multiplicityWeight = 1.0;


            RigidBodyFunctions::initMassMatrixInv<TRigidBody>(body);
    };

};

#endif // RigidBodyFunctionsMPI_hpp

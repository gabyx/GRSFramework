
#include "DynamicsSystemMPI.hpp"

#include "AddGyroTermVisitor.hpp"
#include "VectorToSkewMatrix.hpp"
#include "CommonFunctions.hpp"

DynamicsSystem::~DynamicsSystem() {
    DECONSTRUCTOR_MESSAGE

    // Delete all RigidBodys
    m_SimBodies.removeAndDeleteAllBodies();
    m_RemoteSimBodies.removeAndDeleteAllBodies();
    m_Bodies.removeAndDeleteAllBodies();

};


void DynamicsSystem::getSettings(RecorderSettings & SettingsRecorder) const{
    SettingsRecorder = m_SettingsRecorder;
}


void DynamicsSystem::setSettings(const RecorderSettings & SettingsRecorder){
    m_SettingsRecorder = SettingsRecorder;
}


void DynamicsSystem::getSettings(TimeStepperSettings &SettingsTimestepper, InclusionSolverSettings &SettingsInclusionSolver) {
    SettingsTimestepper = m_SettingsTimestepper;
    SettingsInclusionSolver = m_SettingsInclusionSolver;
}


void DynamicsSystem::setSettings(const TimeStepperSettings &SettingsTimestepper, const InclusionSolverSettings &SettingsInclusionSolver) {
    m_SettingsTimestepper = SettingsTimestepper;
    m_SettingsInclusionSolver = SettingsInclusionSolver;
}


void DynamicsSystem::initializeLog(Logging::Log* pLog) {
    m_pSolverLog = pLog;
    ASSERTMSG(m_pSolverLog != NULL, "Logging::Log: NULL!");
}


void DynamicsSystem::reset(){
    //reset all external forces
    m_externalForces.reset();
    initMassMatrixAndHTerm();
}


void DynamicsSystem::doFirstHalfTimeStep(PREC ts, PREC timestep) {
    using namespace std;
     #if CoutLevelSolver>1
    LOG(m_pSolverLog, "---> doFirstHalfTimeStep(): "<<std::endl;)
    #endif
    static Matrix43 F_i = Matrix43::Zero();

     m_externalForces.setTime(ts+timestep);

    // Do timestep for every object
    typename RigidBodySimContainerType::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "\t--->Body: "<< RigidBodyId::getBodyIdString(pBody)<< std::endl
            << "\t\t--->m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "\t\t--->m_q_s= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t = ts + timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_back.head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_back.tail<3>();

        // Update Transformation A_IK
        setRotFromQuaternion<>(pBody->m_q_KI,  pBody->m_A_IK);

        // Add in to h-Term ==========
        pBody->m_h_term = pBody->m_h_term_const;
//        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
//            << "m_h_term= "  <<pBody->m_h_term<<std::endl)
        // =========================

        // Term omega x Theta * omega = if Theta is diagonal : for a Spehere for example!
        AddGyroTermVisitor vis(pBody);

        // Add in to Mass Matrix
        // Mass Matrix is Constant!
        // =================

        // Add external forces to h_term
        m_externalForces.calculate(pBody);


#if CoutLevelSolver>2
        LOG(m_pSolverLog, "\t--->Body: "<< RigidBodyId::getBodyIdString(pBody)<< std::endl
            << "\t\t--->m_t= "  << pBody->m_pSolverData->m_t<<std::endl
            << "\t\t--->m_q_m= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
    }

}


void DynamicsSystem::doSecondHalfTimeStep(PREC te, PREC timestep) {
    using namespace std;
     #if CoutLevelSolver>1
    LOG(m_pSolverLog, "---> doSecondHalfTimeStep(): "<<std::endl;)
    #endif
    static Matrix43 F_i = Matrix43::Zero();

    m_CurrentStateEnergy = 0;
    // Do timestep for every object
    typename RigidBodySimContainerType::iterator  bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);
#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
            << "m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "m_q_e= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t = te;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_front.head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_front.tail<3>();

        //Normalize Quaternion
        pBody->m_q_KI.normalize();


#if OUTPUT_SYSTEMDATA_FILE == 1
        // Calculate Energy
        m_CurrentStateEnergy += 0.5* pBody->m_pSolverData->m_uBuffer.m_front.transpose() * pBody->m_MassMatrix_diag.asDiagonal() * pBody->m_pSolverData->m_uBuffer.m_front;
        m_CurrentStateEnergy -= +  pBody->m_mass *  pBody->m_r_S.transpose() * m_gravity*m_gravityDir ;
#endif

        // Swap uBuffer and reset Front
        pBody->m_pSolverData->swapBuffer();
        pBody->m_pSolverData->reset();


    }

}

void DynamicsSystem::updateFMatrix(const Quaternion & q, Matrix43 & F_i) {
    static Matrix33 a_tilde = Matrix33::Zero();

    F_i.block<1,3>(0,0) = -0.5 * q.tail<3>();
    updateSkewSymmetricMatrix<>(q.tail<3>(), a_tilde );
    F_i.block<3,3>(1,0) = 0.5 * ( Matrix33::Identity() * q(0) + a_tilde );
}


void DynamicsSystem::initMassMatrixAndHTerm() {
    // iterate over all objects and assemble matrix M
    typename RigidBodySimContainerType::iterator bodyIt;

    Vector3 gravity = m_gravity * m_gravityDir;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {
        RigidBodyFunctions::initMassMatrixAndHTerm( *bodyIt, gravity);
    }
}


#include "DynamicsSystem.hpp"

#include "VectorToSkewMatrix.hpp"
#include "CommonFunctions.hpp"

DynamicsSystem::DynamicsSystem() {


};

DynamicsSystem::~DynamicsSystem() {
    DECONSTRUCTOR_MESSAGE
};


void DynamicsSystem::init() {
    initializeGlobalParameters();
}


void DynamicsSystem::initializeGlobalParameters() {
    //m_mass = 0.050;
    m_gravity = 9.81;
    m_gravityDir = Vector3(0,0,-1);
    /* m_ThetaS_A = 2.0/5.0 * m_mass * (m_R*m_R);
     m_ThetaS_B = 2.0/5.0 * m_mass * (m_R*m_R);
     m_ThetaS_C = 2.0/5.0 * m_mass * (m_R*m_R);*/
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

}


void DynamicsSystem::doFirstHalfTimeStep(PREC timestep) {
    using namespace std;

    static Matrix43 F_i = Matrix43::Zero();

    // Do timestep for every object
    std::vector<boost::shared_ptr<MyRigidBodyType> >::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        MyRigidBodyType * pBody = (*bodyIt).get();

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< pBody->m_id <<"-----"<< std::endl
            << "m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "m_q_s= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t += timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_Back.head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_Back.tail<3>();

        // Update Transformation A_IK
        setRotFromQuaternion<>(pBody->m_q_KI,  pBody->m_A_IK);

        // Add in to h-Term ==========
        pBody->m_h_term = pBody->m_h_term_const;
        // Term omega x Theta * omega = 0, because theta is diagonal
        // =========================

        // Add in to Mass Matrix
        // Mass Matrix is Constant!
        // =================

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< pBody->m_id <<"-----" std::endl
            << "m_t= "  << pBody->m_pSolverData->m_t<<std::endl
            << "m_q_m= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
    }
}


void DynamicsSystem::doSecondHalfTimeStep(PREC timestep) {
    using namespace std;

    static Matrix43 F_i = Matrix43::Zero();

    // Do timestep for every object
    std::vector<boost::shared_ptr<MyRigidBodyType> >::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        MyRigidBodyType * pBody = (*bodyIt).get();
#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< pBody->m_id <<"-----"<< std::endl
            << "m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "m_q_e= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t += timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_Front.head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_Front.tail<3>();

        // Swap uuffer and reset Front
        pBody->m_pSolverData->swapBuffer();
        pBody->m_pSolverData->reset();

        //Normalize Quaternion
        pBody->m_q_KI.normalize();


#if OUTPUT_SYSTEMDATA_FILE == 1
        // Calculate Energy
        m_CurrentStateEnergy += 0.5* pBody->m_pSolverData->m_uBuffer.m_Front.transpose() * pBody->m_MassMatrix_diag.asDiagonal() * pBody->m_pSolverData->m_uBuffer.m_Front;
        m_CurrentStateEnergy -= +  pBody->m_mass *  pBody->m_r_S.transpose() * m_gravity*m_gravityDir ;
#endif

    }

}

void DynamicsSystem::updateFMatrix(const Quaternion & q, Matrix43 & F_i) {
    static Matrix33 a_tilde = Matrix33::Zero();

    F_i.block<1,3>(0,0) = -0.5 * q.tail<3>();
    updateSkewSymmetricMatrix<>(q.tail<3>(), a_tilde );
    F_i.block<3,3>(1,0) = 0.5 * ( Matrix33::Identity() * q(0) + a_tilde );
}


void DynamicsSystem::init_MassMatrix() {
    // iterate over all objects and assemble matrix M
    for(int i=0; i < m_SimBodies.size(); i++) {
        m_SimBodies[i]->m_MassMatrix_diag.head<3>().setConstant(m_SimBodies[i]->m_mass);
        m_SimBodies[i]->m_MassMatrix_diag.tail<3>() = m_SimBodies[i]->m_K_Theta_S;
    }
}


void DynamicsSystem::init_MassMatrixInv() {
    // iterate over all objects and assemble matrix M inverse
    for(int i=0; i < m_SimBodies.size(); i++) {
        m_SimBodies[i]->m_MassMatrixInv_diag = m_SimBodies[i]->m_MassMatrix_diag.array().inverse().matrix();
    }
}

void DynamicsSystem::init_const_hTerm() {
    // Fill in constant terms of h-Term
    for(int i=0; i < m_SimBodies.size(); i++) {
        m_SimBodies[i]->m_h_term_const.head<3>() =  m_SimBodies[i]->m_mass * m_gravity * m_gravityDir;
    }
}

#include "DynamicsSystem.hpp"

#include "VectorToSkewMatrix.hpp"
#include "AddGyroTermVisitor.hpp"
#include "CommonFunctions.hpp"
#include "RigidBodyFunctions.hpp"


DynamicsSystemBase::DynamicsSystemBase(){
    // set reasonable standart values:

    m_currentTotEnergy = 0;
    m_currentPotEnergy= 0;
    m_currentKinEnergy= 0;
    m_currentTransKinEnergy= 0;
    m_currentRotKinEnergy= 0;
    m_currentSpinNorm= 0;
}

DynamicsSystemBase::~DynamicsSystemBase() {
    DECONSTRUCTOR_MESSAGE

    // Delete all RigidBodys
    m_SimBodies.deleteAllBodies();
    m_Bodies.deleteAllBodies();

};


const DynamicsSystemBase::RecorderSettingsType & DynamicsSystemBase::getSettingsRecorder() const {
    return m_settingsRecorder;
}
const DynamicsSystemBase::TimeStepperSettingsType &
DynamicsSystemBase::getSettingsTimeStepper() const {
    return m_settingsTimestepper;
}
const DynamicsSystemBase::InclusionSolverSettingsType & DynamicsSystemBase::getSettingsInclusionSolver() const {
    return m_settingsInclusionSolver;
}

void DynamicsSystemBase::setSettings(const TimeStepperSettingsType &settingsTimestepper){
    m_settingsTimestepper = settingsTimestepper;
}
void DynamicsSystemBase::setSettings(const RecorderSettingsType & settingsRecorder) {
    m_settingsRecorder = settingsRecorder;
}
void DynamicsSystemBase::setSettings(const InclusionSolverSettingsType &settingsInclusionSolver){
    m_settingsInclusionSolver = settingsInclusionSolver;
}

void DynamicsSystemBase::getSettings(TimeStepperSettingsType &settingsTimestepper,
                                 InclusionSolverSettingsType &settingsInclusionSolver) const {
    settingsTimestepper = m_settingsTimestepper;
    settingsInclusionSolver = m_settingsInclusionSolver;
}
void DynamicsSystemBase::setSettings(const TimeStepperSettingsType &settingsTimestepper,
                                 const InclusionSolverSettingsType &settingsInclusionSolver) {
    m_settingsTimestepper = settingsTimestepper;
    m_settingsInclusionSolver = settingsInclusionSolver;
}


void DynamicsSystemBase::initializeLog(Logging::Log* pLog) {
    m_pSolverLog = pLog;
    ASSERTMSG(m_pSolverLog != nullptr, "Logging::Log: nullptr!");
}


void DynamicsSystemBase::reset() {
    //reset all external forces
    m_externalForces.reset();

    initMassMatrixAndHTerm();
}


void DynamicsSystemBase::doFirstHalfTimeStep(PREC ts, PREC timestep) {
    using namespace std;
#if CoutLevelSolver>1
    LOG(m_pSolverLog, "---> doFirstHalfTimeStep(): "<<std::endl;)
#endif

    static Matrix43 F_i = Matrix43::Zero();

    m_externalForces.setTime(ts+timestep);

    // Do timestep for every object
    for(auto bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "\t---> Body: "<< RigidBodyId::getBodyIdString(pBody) << std::endl
            << "\t\t--->m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "\t\t--->m_q_s= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl
            << "\t\t--->m_u_s= "  <<pBody->m_pSolverData->m_uBuffer.m_back.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t = ts + timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_back.head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_back.tail<3>();

        //Normalize Quaternion
        pBody->m_q_KI.normalize();

        // Update Transformation A_IK
        QuaternionHelpers::setRotFromQuaternion<>(pBody->m_q_KI,  pBody->m_A_IK);

        // Add in to h-Term ==========
        pBody->m_h_term.setZero();

        // Term omega x Theta * omega = if Theta is diagonal : for a Spehere for example!
        AddGyroTermVisitor vis(pBody);

        // =========================

        // Add in to Mass Matrix
        // Mass Matrix is Constant!
        // =================

        // Add external forces to h_term
        m_externalForces.calculate(pBody);



#if CoutLevelSolver>2
        LOG(m_pSolverLog, "\t--->Body: "<< RigidBodyId::getBodyIdString(pBody) <<std::endl
            << "\t\t--->m_t= "  << pBody->m_pSolverData->m_t<<std::endl
            << "\t\t--->m_q_m= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
    }
}


void DynamicsSystemBase::doSecondHalfTimeStep(PREC te, PREC timestep) {
    using namespace std;
#if CoutLevelSolver>1
    LOG(m_pSolverLog, "---> doSecondHalfTimeStep(): "<<std::endl;)
#endif
    static Matrix43 F_i = Matrix43::Zero();

    m_currentTotEnergy = 0;

    // Do timestep for every object
    typename RigidBodySimContainerType::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);

        // Update time:
        pBody->m_pSolverData->m_t = te;

        // We do this: Symmetric update q_E = q_B + deltaT * F(q_M)(u_E+u_B)/2 (only influences rotation)
        // Original Moreau update    q_E = q_B + deltaT * F(q_M)u_E/2  + F(q_B) u_B/2

        // Set F(q_m) for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_front.head<3>();
        pBody->m_q_KI += timestep * F_i * (pBody->m_pSolverData->m_uBegin.tail<3>() + pBody->m_pSolverData->m_uBuffer.m_front.tail<3>());

        //Normalize Quaternion
        pBody->m_q_KI.normalize();

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "\t--->Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
            << "\t\t--->m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "\t\t--->m_q_e= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl
            << "\t\t--->m_u_e= "  <<pBody->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl;)
#endif

#if OUTPUT_SIMDATA_FILE == 1
        // Calculate Energy
        PREC potE = m_externalForces.calculatePotEnergy(pBody);
        PREC kinE = 0.5* pBody->m_pSolverData->m_uBuffer.m_front.transpose() * pBody->m_MassMatrix_diag.asDiagonal() * pBody->m_pSolverData->m_uBuffer.m_front;
        PREC transKinE = 0.5*pBody->m_pSolverData->m_uBuffer.m_front.squaredNorm()*pBody->m_mass;
        m_currentPotEnergy += potE;
        m_currentKinEnergy += kinE;
        m_currentTransKinEnergy += transKinE;
        m_currentRotKinEnergy += kinE - transKinE;
        m_currentTotEnergy += kinE + potE;
        m_currentSpinNorm = (pBody->m_K_Theta_S.asDiagonal() * pBody->m_pSolverData->m_uBuffer.m_front.tail<3>()).norm();
#endif


        // Swap uBuffer and reset Front
        pBody->m_pSolverData->swapBuffer();
        pBody->m_pSolverData->reset();
    }
}

void DynamicsSystemBase::updateFMatrix(const Quaternion & q, Matrix43 & F_i) {
    static Matrix33 a_tilde = Matrix33::Zero();

    F_i.block<1,3>(0,0) = -0.5 * q.tail<3>();
    updateSkewSymmetricMatrix<>(q.tail<3>(), a_tilde );
    F_i.block<3,3>(1,0) = 0.5 * ( Matrix33::Identity() * q(0) + a_tilde );
}


void DynamicsSystemBase::initMassMatrixAndHTerm() {
    // iterate over all objects and assemble matrix M
    typename RigidBodySimContainerType::iterator bodyIt;

    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {
        RigidBodyFunctions::initMassMatrixAndHTerm( *bodyIt);
    }
}


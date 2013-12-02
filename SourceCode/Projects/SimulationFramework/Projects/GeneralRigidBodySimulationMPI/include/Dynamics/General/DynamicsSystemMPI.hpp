﻿#ifndef DynamicsSystemMPI_hpp
#define DynamicsSystemMPI_hpp

#include <vector>
#include <list>
#include <Eigen/Core>
#include <Eigen/LU>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBodyMPI.hpp"
#include "RigidBodyContainer.hpp"

#include "ContactParameterMap.hpp"
#include "AddGyroTermVisitor.hpp"
#include "DynamicsState.hpp"
#include "InitialConditionBodies.hpp"
#include "CommonFunctions.hpp"

#include "InclusionSolverSettings.hpp"
#include "TimeStepperSettings.hpp"



#include "MPICommunication.hpp"
#include "NeighbourCommunicator.hpp"

#include "SimpleLogger.hpp"

#include "ExternalForces.hpp"

template<typename TDynamicsSystemConfig>
class DynamicsSystem {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef TDynamicsSystemConfig DynamicsSystemConfig;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemConfig)


    DynamicsSystem();
    ~DynamicsSystem();

    // General related variables
    double m_gravity;
    Vector3 m_gravityDir;

    ContactParameterMap<RigidBodyType> m_ContactParameterMap;

    typedef ExternalForceList<DynamicsSystem> ExternalForceListType;
    ExternalForceListType m_externalForces; ///< Special class of function objects

    //All Global Geometries used in the System
    typedef std::map< unsigned int /* id */, typename RigidBodyType::GeometryType> GlobalGeometryMapType;
    GlobalGeometryMapType m_globalGeometries;

    // All global RigidBodies Container for this Process, these bodies which are owned by this class!"============================
    typedef RigidBodyContainer<RigidBodyType> RigidBodySimContainerType;
    RigidBodySimContainerType m_SimBodies;        // simulated objects
    RigidBodySimContainerType m_RemoteSimBodies;  // all remote bodies

    typedef RigidBodySimContainerType RigidBodyNotAniContainer;
    RigidBodyNotAniContainer m_Bodies;        // all not simulated objects
    // ============================================================================




    inline void addSimBodyPtr(RigidBodyType * ptr ) { m_SimBodies.insert(ptr); }
    inline void addBodyPtr(RigidBodyType * ptr ) { m_Bodies.push_back(ptr); }

    void initializeLog(Logging::Log* pLog);

    void initMassMatrixAndHTerm();


    void doFirstHalfTimeStep(PREC ts, PREC timestep);
    void doSecondHalfTimeStep(PREC te, PREC timestep);

    void getSettings(RecorderSettings & SettingsRecorder) const;
    void setSettings(const RecorderSettings & SettingsRecorder);
    void getSettings(TimeStepperSettings &SettingsTimestepper, InclusionSolverSettings &SettingsInclusionSolver);
    void setSettings(const TimeStepperSettings &SettingsTimestepper, const InclusionSolverSettings &SettingsInclusionSolver);

    void reset();
    inline  void afterFirstTimeStep() {};
    inline  void afterSecondTimeStep() {};
    void doInputTimeStep(PREC T) {};

    double m_CurrentStateEnergy;

protected:


    RecorderSettings m_SettingsRecorder;
    TimeStepperSettings m_SettingsTimestepper;
    InclusionSolverSettings m_SettingsInclusionSolver;


    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);


    // Log
    Logging::Log*	m_pSolverLog;
    std::stringstream logstream;

};




#include "VectorToSkewMatrix.hpp"


template<typename TDynamicsSystemConfig>
DynamicsSystem<TDynamicsSystemConfig>::DynamicsSystem()
{
};

template<typename TDynamicsSystemConfig>
DynamicsSystem<TDynamicsSystemConfig>::~DynamicsSystem() {
    DECONSTRUCTOR_MESSAGE

    // Delete all RigidBodys
    m_SimBodies.removeAndDeleteAllBodies();
    m_RemoteSimBodies.removeAndDeleteAllBodies();
    m_Bodies.removeAndDeleteAllBodies();

};

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::getSettings(RecorderSettings & SettingsRecorder) const{
    SettingsRecorder = m_SettingsRecorder;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::setSettings(const RecorderSettings & SettingsRecorder){
    m_SettingsRecorder = SettingsRecorder;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::getSettings(TimeStepperSettings &SettingsTimestepper, InclusionSolverSettings &SettingsInclusionSolver) {
    SettingsTimestepper = m_SettingsTimestepper;
    SettingsInclusionSolver = m_SettingsInclusionSolver;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::setSettings(const TimeStepperSettings &SettingsTimestepper, const InclusionSolverSettings &SettingsInclusionSolver) {
    m_SettingsTimestepper = SettingsTimestepper;
    m_SettingsInclusionSolver = SettingsInclusionSolver;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::initializeLog(Logging::Log* pLog) {
    m_pSolverLog = pLog;
    ASSERTMSG(m_pSolverLog != NULL, "Logging::Log: NULL!");
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::reset(){
    //reset all external forces
    m_externalForces.reset();
    initMassMatrixAndHTerm();
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::doFirstHalfTimeStep(PREC ts, PREC timestep) {
    using namespace std;

    static Matrix43 F_i = Matrix43::Zero();

     m_externalForces.setTime(ts+timestep);

    // Do timestep for every object
    typename RigidBodySimContainerType::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
            << "m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "m_q_s= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t = ts + timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_back.template head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_back.template tail<3>();

        // Update Transformation A_IK
        setRotFromQuaternion<>(pBody->m_q_KI,  pBody->m_A_IK);

        // Add in to h-Term ==========
        pBody->m_h_term = pBody->m_h_term_const;
//        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
//            << "m_h_term= "  <<pBody->m_h_term<<std::endl)
        // =========================

        // Term omega x Theta * omega = if Theta is diagonal : for a Spehere for example!
        AddGyroTermVisitor<RigidBodyType> vis(pBody);

        // Add in to Mass Matrix
        // Mass Matrix is Constant!
        // =================

        // Add external forces to h_term
        m_externalForces.calculate(pBody);


#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
            << "m_t= "  << pBody->m_pSolverData->m_t<<std::endl
            << "m_q_m= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
    }

}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::doSecondHalfTimeStep(PREC te, PREC timestep) {
    using namespace std;

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
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_front.template head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_front.template tail<3>();

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
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::updateFMatrix(const Quaternion & q, Matrix43 & F_i) {
    static Matrix33 a_tilde = Matrix33::Zero();

    F_i.template block<1,3>(0,0) = -0.5 * q.template tail<3>();
    updateSkewSymmetricMatrix<>(q.template tail<3>(), a_tilde );
    F_i.template block<3,3>(1,0) = 0.5 * ( Matrix33::Identity() * q(0) + a_tilde );
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::initMassMatrixAndHTerm() {
    // iterate over all objects and assemble matrix M
    typename RigidBodySimContainerType::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        //Mass Matrix
        (*bodyIt)->m_MassMatrix_diag.template head<3>().setConstant((*bodyIt)->m_mass);
        (*bodyIt)->m_MassMatrix_diag.template tail<3>() = (*bodyIt)->m_K_Theta_S;

        // Massmatrix Inverse
        (*bodyIt)->m_MassMatrixInv_diag = (*bodyIt)->m_MassMatrix_diag.array().inverse().matrix();
        // H_const term
        (*bodyIt)->m_h_term_const.template head<3>() =  (*bodyIt)->m_mass * m_gravity * m_gravityDir;
    }
}


#endif

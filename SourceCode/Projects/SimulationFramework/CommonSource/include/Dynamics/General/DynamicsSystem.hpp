#ifndef DynamicsSystem_hpp
#define DynamicsSystem_hpp

#include <vector>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"
#include "RigidBodyContainer.hpp"
#include "DynamicsState.hpp"
#include "ContactParameterMap.hpp"

#include "AddGyroTermVisitor.hpp"
#include "InitialConditionBodies.hpp"
#include "InclusionSolverSettings.hpp"
#include "CommonFunctions.hpp"

#include "RecorderSettings.hpp"
#include "TimeStepperSettings.hpp"

#include "SimpleLogger.hpp"



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

    //All Global Geometries used in the System
    typedef std::map< unsigned int /* id */, typename RigidBodyType::GeometryType> GlobalGeometryMapType;
    GlobalGeometryMapType m_globalGeoms;

    // All RigidBodies which are owned by this class!
    typedef RigidBodyContainer<typename RigidBodyType::RigidBodyIdType,RigidBodyType> RigidBodySimContainer;
    RigidBodySimContainer m_SimBodies;    // Simulated Objects
    typedef RigidBodyContainer<typename RigidBodyType::RigidBodyIdType,RigidBodyType> RigidBodyNotAniContainer;
    RigidBodyNotAniContainer m_Bodies;    // all not simulated objects

    void initializeLog(Logging::Log* pLog);


    inline void applySimBodiesToDynamicsState(DynamicsState<LayoutConfigType> & state);
    inline void applyDynamicsStateToSimBodies(const DynamicsState<LayoutConfigType> & state);

    void init_MassMatrix(); // MassMatrix is const
    void init_MassMatrixInv(); // MassMatrix is const
    void init_const_hTerm(); // Initializes constant terms in h

    //Virtuals
    void doFirstHalfTimeStep( PREC timestep);
    void doSecondHalfTimeStep( PREC timestep);

    void getSettings(RecorderSettings<LayoutConfigType> & SettingsRecorder) const;
    void setSettings(const RecorderSettings<LayoutConfigType> & SettingsRecorder);
    void getSettings(TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver) const;
    void setSettings(const TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, const InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver);

    void reset();
    inline  void afterFirstTimeStep() {};
    inline  void afterSecondTimeStep() {};
    void doInputTimeStep(PREC T) {};

    double m_CurrentStateEnergy;

protected:

    RecorderSettings<LayoutConfigType> m_SettingsRecorder;
    TimeStepperSettings<LayoutConfigType> m_SettingsTimestepper;
    InclusionSolverSettings<LayoutConfigType> m_SettingsInclusionSolver;

    //Function
    //This is a minimal update of F, no checking if constant values are correct
    void updateFMatrix(const Quaternion & q, Matrix43 & F_i);


    // Log
    Logging::Log*	m_pSolverLog;
    std::stringstream logstream;
};




#include "VectorToSkewMatrix.hpp"


template<typename TDynamicsSystemConfig>
DynamicsSystem<TDynamicsSystemConfig>::DynamicsSystem() {

};
template<typename TDynamicsSystemConfig>
DynamicsSystem<TDynamicsSystemConfig>::~DynamicsSystem() {
    DECONSTRUCTOR_MESSAGE

    // Delete all RigidBodys
    {
        typename RigidBodySimContainer::iterator it;
        for(it = m_SimBodies.begin(); it != m_SimBodies.end(); it++){
            delete (*it);
        }
    }

    {
        typename RigidBodyNotAniContainer::iterator it;
        for(it = m_Bodies.begin(); it != m_Bodies.end(); it++){
            delete (*it);
        }
    }

    m_SimBodies.clear();
    m_Bodies.clear();

};

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::getSettings(RecorderSettings<LayoutConfigType> & SettingsRecorder) const{
    SettingsRecorder = m_SettingsRecorder;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::setSettings(const RecorderSettings<LayoutConfigType> & SettingsRecorder){
    m_SettingsRecorder = SettingsRecorder;
}
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::getSettings(TimeStepperSettings<LayoutConfigType> &SettingsTimestepper,
                                                        InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver) const{
    SettingsTimestepper = m_SettingsTimestepper;
    SettingsInclusionSolver = m_SettingsInclusionSolver;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::setSettings(const TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, const InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver) {
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

}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::applySimBodiesToDynamicsState(DynamicsState<LayoutConfigType> & state){
    InitialConditionBodies::applyBodiesToDynamicsState<RigidBodyType, RigidBodySimContainer>(m_SimBodies,state);
}
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::applyDynamicsStateToSimBodies(const DynamicsState<LayoutConfigType> & state){
    InitialConditionBodies::applyDynamicsStateToBodies<RigidBodyType, RigidBodySimContainer>(state,m_SimBodies);
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::doFirstHalfTimeStep(PREC timestep) {
    using namespace std;

    static Matrix43 F_i = Matrix43::Zero();

    // Do timestep for every object
    typename RigidBodySimContainer::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
            << "m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "m_q_s= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t += timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_Back.template head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_Back.template tail<3>();

        // Update Transformation A_IK
        setRotFromQuaternion<>(pBody->m_q_KI,  pBody->m_A_IK);

        // Add in to h-Term ==========
        pBody->m_h_term = pBody->m_h_term_const;

        // Term omega x Theta * omega = if Theta is diagonal : for a Spehere for example!
        AddGyroTermVisitor<RigidBodyType> vis(pBody);

        // =========================

        // Add in to Mass Matrix
        // Mass Matrix is Constant!
        // =================

#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody)<<"-----" std::endl
            << "m_t= "  << pBody->m_pSolverData->m_t<<std::endl
            << "m_q_m= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
    }
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::doSecondHalfTimeStep(PREC timestep) {
    using namespace std;

    static Matrix43 F_i = Matrix43::Zero();

    m_CurrentStateEnergy = 0;

    // Do timestep for every object
    typename RigidBodySimContainer::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++) {

        RigidBodyType * pBody = (*bodyIt);
#if CoutLevelSolver>2
        LOG(m_pSolverLog, "Body: "<< RigidBodyId::getBodyIdString(pBody) <<"-----"<< std::endl
            << "m_t= "  <<pBody->m_pSolverData->m_t<<std::endl
            << "m_q_e= "  <<pBody->m_r_S.transpose() << "\t"<<pBody->m_q_KI.transpose()<<std::endl;)
#endif
        // Update time:
        pBody->m_pSolverData->m_t += timestep;

        // Set F for this object:
        updateFMatrix(pBody->m_q_KI, F_i);

        // Timestep for position;
        pBody->m_r_S  += timestep * pBody->m_pSolverData->m_uBuffer.m_Front.template head<3>();
        pBody->m_q_KI += timestep * F_i * pBody->m_pSolverData->m_uBuffer.m_Front.template tail<3>();

        //Normalize Quaternion
        pBody->m_q_KI.normalize();

#if OUTPUT_SYSTEMDATA_FILE == 1
        // Calculate Energy
        m_CurrentStateEnergy += 0.5* pBody->m_pSolverData->m_uBuffer.m_Front.transpose() * pBody->m_MassMatrix_diag.asDiagonal() * pBody->m_pSolverData->m_uBuffer.m_Front;
        m_CurrentStateEnergy -= +  pBody->m_mass *  pBody->m_r_S.transpose() * m_gravity*m_gravityDir ;
#endif


        // Swap uuffer and reset Front
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
void DynamicsSystem<TDynamicsSystemConfig>::init_MassMatrix() {
    // iterate over all objects and assemble matrix M
    typename RigidBodySimContainer::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++){
        (*bodyIt)->m_MassMatrix_diag.template head<3>().setConstant((*bodyIt)->m_mass);
         (*bodyIt)->m_MassMatrix_diag.template tail<3>() = (*bodyIt)->m_K_Theta_S;
    }
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::init_MassMatrixInv() {
    // iterate over all objects and assemble matrix M inverse
    typename RigidBodySimContainer::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++){
        (*bodyIt)->m_MassMatrixInv_diag = (*bodyIt)->m_MassMatrix_diag.array().inverse().matrix();
    }
}
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::init_const_hTerm() {
    // Fill in constant terms of h-Term
    typename RigidBodySimContainer::iterator bodyIt;
    for(bodyIt = m_SimBodies.begin() ; bodyIt != m_SimBodies.end(); bodyIt++){
         (*bodyIt)->m_h_term_const.template head<3>() =  (*bodyIt)->m_mass * m_gravity * m_gravityDir;
    }
}


#endif

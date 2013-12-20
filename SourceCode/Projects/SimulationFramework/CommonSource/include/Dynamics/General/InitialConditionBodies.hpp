#ifndef InitialConditionBodies_hpp
#define InitialConditionBodies_hpp

#include <boost/filesystem.hpp>

#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"


namespace InitialConditionBodies {

void setupPositionBodiesLinear(
    DynamicsState & init_state,
    typename DynamicsState::Vector3 pos,
    typename DynamicsState::Vector3 dir,
    double dist, bool jitter, double delta, unsigned int seed);

void setupPositionBodiesGrid(DynamicsState & init_state,
                     unsigned int gDim_x,
                     unsigned int gDim_y,
                     double d,
                     typename DynamicsState::Vector3 vec_trans,
                     bool jitter,
                     double delta,
                     unsigned int seed);

bool setupPositionBodiesFromFile(DynamicsState & init_state, boost::filesystem::path file_path);

void setupPositionBodyPosAxisAngle(RigidBodyState & rigibodyState,
                                   const typename RigidBodyState::Vector3 & pos,
                                   typename RigidBodyState::Vector3 & axis,
                                   typename RigidBodyState::PREC angleRadian);


template<typename TRigidBodyType,  typename TRigidBodyList>
inline void applyDynamicsStateToBodies(
                                       const DynamicsState & state,
                                       TRigidBodyList & bodies) {

    typedef typename TRigidBodyType::LayoutConfigType LayoutConfigType;

    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );

    typename  TRigidBodyList::iterator bodyIt;
    typename  DynamicsState::RigidBodyStateListType::const_iterator stateBodyIt = state.m_SimBodyStates.begin();

    for(bodyIt = bodies.begin(); bodyIt != bodies.end() ; bodyIt++) {

        (*bodyIt)->m_r_S  = stateBodyIt->m_q.template head<3>();
        (*bodyIt)->m_q_KI = stateBodyIt->m_q.template tail<4>();

        if( (*bodyIt)->m_eState == TRigidBodyType::BodyState::SIMULATED) {
            (*bodyIt)->m_pSolverData->m_uBuffer.m_back = stateBodyIt->m_u;
            (*bodyIt)->m_pSolverData->m_t = state.m_t;
        }

        (*bodyIt)->m_A_IK= getRotFromQuaternion<typename LayoutConfigType::PREC>((*bodyIt)->m_q_KI);
        stateBodyIt++;
    }
}

// Prototype
template<typename TRigidBody, typename TRigidBodyState>
inline void applyBodyToRigidBodyState( const TRigidBody  * body, TRigidBodyState & rigidBodyState );

template<typename TRigidBodyType, typename TRigidBodyList>
inline void applyBodiesToDynamicsState(const TRigidBodyList & bodies,
                                         DynamicsState & state ) {

    typedef typename TRigidBodyType::LayoutConfigType LayoutConfigType;

    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );
    typename  TRigidBodyList::const_iterator bodyIt = bodies.begin();
    typename  DynamicsState::RigidBodyStateListType::iterator stateBodyIt = state.m_SimBodyStates.begin();

    for(bodyIt = bodies.begin(); bodyIt != bodies.end() ; bodyIt++) {
        //std::cout << RigidBodyId::getBodyIdString(*bodyIt) << std::cout;
        applyBodyToRigidBodyState( (*bodyIt), (*stateBodyIt) );
        stateBodyIt++;
    }
}


template<typename TRigidBody, typename TRigidBodyState>
inline void applyBodyToRigidBodyState( const TRigidBody  * body, TRigidBodyState & rigidBodyState ) {

    rigidBodyState.m_q.template head<3>() = body->m_r_S;
    rigidBodyState.m_q.template tail<4>() = body->m_q_KI;

    if(body->m_pSolverData) {
        rigidBodyState.m_u = body->m_pSolverData->m_uBuffer.m_back;
    } else {
        ASSERTMSG(false,"Your rigid body has no data in m_pSolverData (for velocity), this operation might be incorret!")
        rigidBodyState.m_u.setZero();
    }

}


};

#endif

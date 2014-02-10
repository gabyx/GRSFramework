#ifndef InitialConditionBodies_hpp
#define InitialConditionBodies_hpp

#include <boost/filesystem.hpp>

#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

namespace InitialConditionBodies {

DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

template<typename StateContainerType>
void setupPositionBodiesLinear(
    StateContainerType & states,
    typename DynamicsState::Vector3 pos,
    typename DynamicsState::Vector3 dir,
    double dist, bool jitter, double delta, unsigned int seed){

    DEFINE_LAYOUT_CONFIG_TYPES


    dir.normalize();
    Vector3 jitter_vec, random_vec;
    jitter_vec.setZero();

    // Set only m_q, m_u is zero in constructor!
    double d = 2; //spread around origin with 0.5m
    unsigned int i = 0;
    for(auto itState = states.begin(); itState != states.end(); itState++) {
        auto & state = itState->second;
        state.m_q.template tail<4>() = Quaternion(1,0,0,0);

        typedef boost::mt19937  RNG;
        static  RNG generator(seed);
        static  boost::uniform_real<PREC> uniform(-1.0,1.0);
        static  boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

        if(jitter) {
            random_vec = Vector3(randomNumber(),randomNumber(),randomNumber()); // No uniform distribution!, but does not matter
            random_vec.normalize();
            random_vec = random_vec.cross(dir);
            random_vec.normalize();
            jitter_vec = random_vec * delta;
        }

       state.m_q.template head<3>() = pos + dir*dist*i + jitter_vec;
       i++;
    }
}

template<typename StateContainerType>
void setupPositionBodiesGrid(StateContainerType & states,
                             unsigned int gDim_x,
                             unsigned int gDim_y,
                             double d,
                             typename DynamicsState::Vector3 vec_trans,
                             bool jitter,
                             double delta,
                             unsigned int seed){

    DEFINE_LAYOUT_CONFIG_TYPES

    Vector3 jitter_vec;
    jitter_vec.setZero();
    unsigned int i = 0;
    for(auto itState = states.begin(); itState != states.end(); itState++) {
        auto & state = itState->second;
        state.m_q.template tail<4>() = Quaternion(1,0,0,0);
        int index_z = (i /(gDim_x*gDim_y));
        int index_y = (i - index_z*(gDim_x*gDim_y)) / gDim_x;
        int index_x = (i - index_z*(gDim_x*gDim_y)- index_y*gDim_x);

        typedef boost::mt19937  RNG;
        static RNG generator(seed);
        static boost::uniform_real<PREC> uniform(-1.0,1.0);
        static boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

        if(jitter) {
            jitter_vec = Vector3(randomNumber(),randomNumber(),randomNumber()) * delta; // No uniform distribution!, but does not matter
        }

        state.m_q.template head<3>() = Vector3(index_x * d - 0.5*(gDim_x-1)*d, index_y*d - 0.5*(gDim_y-1)*d , index_z*d) + vec_trans + jitter_vec;
        i++;
    }

}


template<typename StateContainerType>
bool setupPositionBodiesFromFile(StateContainerType & states,
                                 boost::filesystem::path file_path){

    MultiBodySimFile simFile( DynamicsState::LayoutConfigType::LayoutType::NDOFqBody,
                              DynamicsState::LayoutConfigType::LayoutType::NDOFuBody);

    if(simFile.openRead(file_path,states.size())) {
        //simFile >> init_state ;
        simFile.close();
        return true;
    }else{
        ERRORMSG("InitialConditionBodies:: failed: " << simFile.getErrorString())
    }

    return false;
}


void setupPositionBodyPosAxisAngle(RigidBodyState & rigibodyState,
                                   const typename RigidBodyState::Vector3 & pos,
                                   typename RigidBodyState::Vector3 & axis,
                                   typename RigidBodyState::PREC angleRadian);


// Prototype
template<typename TRigidBody, typename TRigidBodyState>
inline void applyRigidBodyStateToBody(const TRigidBodyState & rigidBodyState, TRigidBody  * body );

template<typename RigidBodyContainer, typename RigidBodyStatesContainer>
inline void applyRigidBodyStatesToBodies(RigidBodyContainer & bodies, const RigidBodyStatesContainer & states ){

    for(auto bodyIt = bodies.begin(); bodyIt!= bodies.end(); bodyIt++){
        auto resIt = states.find((*bodyIt)->m_id);
        if( resIt == states.end()){
            ERRORMSG(" There is no initial state for sim body id: " << (*bodyIt)->m_id << std::endl);
        }
        InitialConditionBodies::applyRigidBodyStateToBody( resIt->second, (*bodyIt) );
    }
}

//template<typename TRigidBodyType,  typename TRigidBodyList>
//inline void applyDynamicsStateToBodies(const DynamicsState & state,
//                                       TRigidBodyList & bodies) {
//
//    typedef typename TRigidBodyType::LayoutConfigType LayoutConfigType;
//
//    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );
//
//    typename  TRigidBodyList::iterator bodyIt;
//
//    for(bodyIt = bodies.begin(); bodyIt != bodies.end() ; bodyIt++) {
//
//        unsigned int bodyNr = RigidBodyId::getBodyNr(*bodyIt);
//        ASSERTMSG(bodyNr >=0 && bodyNr < state->m_SimBodyStates.size(), "BodyNr: " << bodyNr << " is out of bound!")
//
//        auto & stateRef = state->m_SimBodyStates[bodyNr];
//
//        applyRigidBodyStateToBody( stateRef, (*bodyIt) );
//
//        if( (*bodyIt)->m_eState == TRigidBodyType::BodyState::SIMULATED) {
//            (*bodyIt)->m_pSolverData->m_t = state.m_t;
//        }
//
//    }
//}

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
    rigidBodyState.m_id = body->m_id;
    rigidBodyState.m_q.template head<3>() = body->m_r_S;
    rigidBodyState.m_q.template tail<4>() = body->m_q_KI;

    if(body->m_pSolverData) {
        rigidBodyState.m_u = body->m_pSolverData->m_uBuffer.m_back;
    } else {
        ASSERTMSG(false,"Your rigid body has no data in m_pSolverData (for velocity), this operation might be incorret!")
        rigidBodyState.m_u.setZero();
    }

}

template<typename TRigidBody, typename TRigidBodyState>
inline void applyRigidBodyStateToBody(const TRigidBodyState & rigidBodyState, TRigidBody  * body ) {
    ASSERTMSG(body->m_id == rigidBodyState.m_id, "Body id is not the same!")
    body->m_r_S = rigidBodyState.m_q.template head<3>();
    body->m_q_KI = rigidBodyState.m_q.template tail<4>();

    body->m_A_IK= getRotFromQuaternion<typename TRigidBody::PREC>(body->m_q_KI);

    if(body->m_pSolverData) {
        body->m_pSolverData->m_uBuffer.m_back = rigidBodyState.m_u ;
    }
}


};

#endif

#ifndef InitialConditionBodies_hpp
#define InitialConditionBodies_hpp

#include <random>
#include <boost/filesystem.hpp>

#include "CommonFunctions.hpp"
#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"

#include "boost/generator_iterator.hpp"

namespace InitialConditionBodies {

DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

template<typename BodyDataContainer, typename BodyStateContainer>
void setupPositionBodiesLinear(
    BodyDataContainer & bodyDataCont,
    BodyStateContainer & bodyStates,
    RigidBodyIdType startId, // used to appropriatly generate random numbers
    Vector3 pos,
    Vector3 dir,
    double dist, bool jitter, double delta, unsigned int seed){

    DEFINE_LAYOUT_CONFIG_TYPES

    dir.normalize();
    Vector3 jitter_vec, random_vec;

    // Set only m_q, m_u is zero in constructor!
    RandomGenType  gen(seed);
    std::uniform_real_distribution<double> uni(-1.0,1.0);
    if(jitter){ random_vec(0) = uni(gen); random_vec(1) = uni(gen); random_vec(2) = uni(gen);}

    auto diffId = startId;
    unsigned int i; // linear index from the front

    auto stateIt = bodyStates.begin();
    for(auto & b : bodyDataCont) {
        i = b.m_id - startId;
        stateIt->m_q.template tail<4>() = Quaternion(1,0,0,0);

        stateIt->m_q.template head<3>() = pos + dir*dist*i + jitter_vec;

        if(jitter) {

            random_vec = Utilities::genRandomVec<PREC>(random_vec,gen,uni, b.m_id - diffId);
            diffId = b.m_id;

            random_vec = random_vec.cross(dir);
            random_vec.normalize();
            stateIt->m_q.template head<3>() +=  random_vec * delta;
        }
        ++stateIt;
    }
}

template<typename BodyDataContainer, typename BodyStateContainer>
void setupPositionBodiesGrid(BodyDataContainer & bodyDataCont,
                             BodyStateContainer & bodyStates,
                             RigidBodyIdType startId,
                             unsigned int gDim_x,
                             unsigned int gDim_y,
                             double d,
                             Vector3 vec_trans,
                             bool jitter,
                             double delta,
                             unsigned int seed,
                             Vector3 dirZ = Vector3(0,0,1),
                             Vector3 dirX = Vector3(1,0,0)){

    DEFINE_LAYOUT_CONFIG_TYPES

    Vector3 random_vec;
    Matrix33 A_IK;

    A_IK.col(1) = dirZ.cross(dirX); A_IK.col(1).normalize();
    A_IK.col(2) = dirZ; A_IK.col(2).normalize();
    A_IK.col(0) = A_IK.col(1).cross(A_IK.col(2)); A_IK.col(0).normalize();

    RandomGenType  gen(seed);
    std::uniform_real_distribution<PREC> uni(-1.0,1.0);
    if(jitter){ random_vec(0) = uni(gen); random_vec(1) = uni(gen); random_vec(2) = uni(gen);}

    auto diffId = startId;
    unsigned int i; // linear index from the front
    auto stateIt = bodyStates.begin();
    for(auto & b : bodyDataCont) {
        auto & state = b.m_initState;
        i = b.m_id  - startId;

        stateIt->m_q.template tail<4>() = Quaternion(1,0,0,0);
        int index_z = (i /(gDim_x*gDim_y));
        int index_y = (i - index_z*(gDim_x*gDim_y)) / gDim_x;
        int index_x = (i - index_z*(gDim_x*gDim_y)- index_y*gDim_x);


        stateIt->m_q.template head<3>() = A_IK * Vector3(index_x * d - 0.5*(gDim_x-1)*d, index_y*d - 0.5*(gDim_y-1)*d , index_z*d) + vec_trans;

        if(jitter) {
            random_vec = Utilities::genRandomVec<PREC>(random_vec,gen,uni, b.m_id - diffId);
            diffId = b.m_id ;

            random_vec.normalize();
            stateIt->m_q.template head<3>() += random_vec * delta;
        }
        ++stateIt;
    }

}


template<typename TBodyStateMap>
bool setupInitialConditionBodiesFromFile(boost::filesystem::path file_path,
                                         TBodyStateMap & bodyDataCont,
                                         double & stateTime,
                                         bool readPos = true,
                                         bool readVel = true,
                                         short which = 2){

    MultiBodySimFile simFile;
    bool failed = false;
    if(simFile.openRead(file_path,
                        DynamicsState::LayoutConfigType::LayoutType::NDOFqBody,
                        DynamicsState::LayoutConfigType::LayoutType::NDOFuBody,
                        0,true))
    {
        // We only perform an update! -> true
        if(!simFile.read(bodyDataCont,stateTime,readPos,readVel,which,true)){
            failed = true;
        }
        simFile.close();
    }else{
        failed = true;
    }

    if(!failed){
        return true;
    }

    ERRORMSG("setupInitialConditionBodiesFromFile:: failed: " << "path: " << file_path << " error:" << simFile.getErrorString())
    return false;
}


void setupPositionBodyPosAxisAngle(RigidBodyState & rigibodyState,
                                   const typename RigidBodyState::Vector3 & pos,
                                   typename RigidBodyState::Vector3 & axis,
                                   typename RigidBodyState::PREC angleRadian);



template<typename RigidBodyContainer, typename RigidBodyStatesContainer>
inline void applyBodyStatesTo(const RigidBodyStatesContainer & states, RigidBodyContainer & bodies ){

    for(auto bodyIt = bodies.begin(); bodyIt!= bodies.end(); ++bodyIt){
        auto resIt = states.find((*bodyIt)->m_id);
        if( resIt == states.end()){
            ERRORMSG(" There is no initial state for sim body id: " << RigidBodyId::getBodyIdString(*bodyIt) << std::endl);
        }
        (*bodyIt)->template applyBodyState<true>(resIt->second);
    }
}


};

#endif

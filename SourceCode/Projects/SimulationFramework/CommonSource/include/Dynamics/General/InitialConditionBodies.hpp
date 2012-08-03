#ifndef InitialConditionBodies_hpp
#define InitialConditionBodies_hpp

#include <boost/filesystem.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"
#include "RigidBody.hpp"
#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

namespace InitialConditionBodies{

  template<typename TLayoutConfig>
  void setupBodiesLinear(
     DynamicsState<TLayoutConfig> & init_state,
     typename TLayoutConfig::Vector3 pos,
     typename TLayoutConfig::Vector3 dir,
     double dist, bool jitter, double delta, unsigned int seed)
   {

     DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)


     dir.normalize();
     Vector3 jitter_vec, random_vec;
     jitter_vec.setZero();

     // Set only m_q, m_u is zero in constructor!
     double d = 2; //spread around origin with 0.5m
     for(unsigned int i=0;i< init_state.m_nSimBodies;i++){
       init_state.m_SimBodyStates[i].m_q.template tail<4>() = typename TLayoutConfig::Quaternion(1,0,0,0);

         typedef boost::mt19937  RNG;
         static  RNG generator(seed);
         static  boost::uniform_real<PREC> uniform(-1.0,1.0);
         static  boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

       if(jitter){
         random_vec = Vector3(randomNumber(),randomNumber(),randomNumber()); // No uniform distribution!, but does not matter
         random_vec.normalize();
         random_vec = random_vec.cross(dir);
         random_vec.normalize();
         jitter_vec = random_vec * delta;
       }

       init_state.m_SimBodyStates[i].m_q.template head<3>() = pos + dir*dist*i + jitter_vec;
     }
   }

template<typename TLayoutConfig>
void setupBodiesGrid(DynamicsState<TLayoutConfig> & init_state,
                     unsigned int gDim_x,
                     unsigned int gDim_y,
                     double d,
                     typename TLayoutConfig::Vector3 vec_trans,
                     bool jitter,
                     double delta,
                     unsigned int seed)
{
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

  Vector3 jitter_vec;
  jitter_vec.setZero();

  for(unsigned int i=0;i< init_state.m_nSimBodies;i++){
    init_state.m_SimBodyStates[i].m_q.template tail<4>() = typename TLayoutConfig::Quaternion(1,0,0,0);
    int index_z = (i /(gDim_x*gDim_y));
    int index_y = (i - index_z*(gDim_x*gDim_y)) / gDim_y;
    int index_x = (i - index_z*(gDim_x*gDim_y)- index_y*gDim_y);
    //cout << index_x<<","<< index_y<<","<< index_z<<endl;
    double x = -d/2 + d/(double)(init_state.m_nSimBodies) * i;

    typedef boost::mt19937  RNG;
    static RNG generator(seed);
    static boost::uniform_real<PREC> uniform(-1.0,1.0);
    static boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

    if(jitter){
      jitter_vec = Vector3(randomNumber(),randomNumber(),randomNumber()) * delta; // No uniform distribution!, but does not matter
    }

    init_state.m_SimBodyStates[i].m_q.template head<3>() = Vector3(index_x * d - 0.5*(gDim_x-1)*d, index_y*d - 0.5*(gDim_y-1)*d , index_z*d) + vec_trans + jitter_vec;
  }

}

template<typename TLayoutConfig>
bool setupBodiesFromFile(DynamicsState<TLayoutConfig> & init_state, boost::filesystem::path file_path){

   MultiBodySimFile<TLayoutConfig> simFile;

   if(simFile.openSimFileRead(file_path,init_state.m_nSimBodies)){;
      simFile >> init_state ;
      simFile.closeSimFile();
      return true;
   }

   return false;
}

template<typename TLayoutConfig>
void setupBodyPositionAxisAngle(RigidBodyState<TLayoutConfig> & rigibodyState, const typename TLayoutConfig::Vector3 & pos, typename TLayoutConfig::Vector3 & axis, typename TLayoutConfig::PREC angleRadian){

   rigibodyState.m_q.template head<3>() = pos;
   setQuaternion(rigibodyState.m_q.template tail<4>(),axis,angleRadian);
}


template<typename TLayoutConfig, typename TRigidBody>
void applyDynamicsStateToBodies(const DynamicsState<TLayoutConfig> & state, std::vector<boost::shared_ptr<TRigidBody > > & bodies){
    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );

    for(int i=0; i < bodies.size(); i++){
      bodies[i]->m_r_S = state.m_SimBodyStates[i].m_q.template head<3>();
      bodies[i]->m_q_KI= state.m_SimBodyStates[i].m_q.template tail<4>();

      if(bodies[i]->m_eState == TRigidBody::SIMULATED){
         bodies[i]->m_pSolverData->m_uBuffer.m_Back = state.m_SimBodyStates[i].m_u;
         bodies[i]->m_pSolverData->m_t = state.m_t;
      }

      bodies[i]->m_A_IK= getRotFromQuaternion<>(bodies[i]->m_q_KI);
    }
}

template<typename TLayoutConfig, typename TRigidBody>
void applyBodiesToDynamicsState(const std::vector<boost::shared_ptr<TRigidBody > > & bodies, DynamicsState<TLayoutConfig> & state ){
    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );

    typename std::vector<boost::shared_ptr<TRigidBody > >::const_iterator bodyIt;
    typename  DynamicsState<TLayoutConfig>::RigidBodyStateListType::iterator stateBodyIt = state.m_SimBodyStates.begin();
    TRigidBody * pBody;

    for(bodyIt = bodies.begin(); bodyIt != bodies.end() ; bodyIt++){
      pBody = (*bodyIt).get();

       stateBodyIt->m_q.template head<3>() = pBody->m_r_S;
       stateBodyIt->m_q.template tail<4>() = pBody->m_q_KI;

      if(pBody->m_pSolverData){
         stateBodyIt->m_u = pBody->m_pSolverData->m_uBuffer.m_Back;
      }

      stateBodyIt++;
    }
}



};

#endif

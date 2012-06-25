#ifndef InitialConditionBodies_hpp
#define InitialConditionBodies_hpp

#include <boost/filesystem.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"
#include "RigidBody.hpp"
#include "DynamicsState.hpp"
#include "MultiBodySimFile.hpp"

namespace InitialConditionBodies{

  template<typename TLayoutConfig>
  void setupBodiesLinear(
     DynamicsState<TLayoutConfig> & init_state,
     typename TLayoutConfig::Vector3 pos,
     typename TLayoutConfig::Vector3 dir,
     double dist, bool jitter, double delta)
   {

     DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)


     dir.normalize();
     Vector3 jitter_vec, random_vec;
     jitter_vec.setZero();

     // Set only m_q, m_u is zero in constructor!
     double d = 2; //spread around origin with 0.5m
     for(unsigned int i=0;i< init_state.m_nSimBodies;i++){
       init_state.m_SimBodyStates[i].m_q.template tail<4>() = typename TLayoutConfig::Quaternion(1,0,0,0);

       if(jitter){
         random_vec = Vector3(randd(-1,1),randd(-1,1),randd(-1,1)); // No uniform distribution!, but does not matter
         random_vec.normalize();
         random_vec = random_vec.cross(dir);
         random_vec.normalize();
         jitter_vec = random_vec * delta;
       }

       init_state.m_SimBodyStates[i].m_q.template head<3>() = pos + dir*dist*i + jitter_vec;
     }
   }

template<typename TLayoutConfig>
void setupBodiesGrid(DynamicsState<TLayoutConfig> & init_state, unsigned int gDim_x, unsigned int gDim_y, double d, typename TLayoutConfig::Vector3 vec_trans, bool jitter, double delta)
{
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

  Vector3 jitter_vec, random_vec;
  jitter_vec.setZero();

  for(unsigned int i=0;i< init_state.m_nSimBodies;i++){
    init_state.m_SimBodyStates[i].m_q.template tail<4>() = typename TLayoutConfig::Quaternion(1,0,0,0);
    int index_z = (i /(gDim_x*gDim_y));
    int index_y = (i - index_z*(gDim_x*gDim_y)) / gDim_y;
    int index_x = (i - index_z*(gDim_x*gDim_y)- index_y*gDim_y);
    //cout << index_x<<","<< index_y<<","<< index_z<<endl;
    double x = -d/2 + d/(double)(init_state.m_nSimBodies) * i;


    if(jitter){
      random_vec = Vector3(randd(-1,1),randd(-1,1),randd(-1,1)); // No uniform distribution!, but does not matter
      jitter_vec = random_vec * delta;
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


template<typename TLayoutConfig>
void applyDynamicsStateToBodies(const DynamicsState<TLayoutConfig> & state, std::vector<boost::shared_ptr<RigidBody<TLayoutConfig> > > & bodies){
    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );

    for(int i=0; i < bodies.size(); i++){
      bodies[i]->m_r_S = state.m_SimBodyStates[i].m_q.template head<3>();
      bodies[i]->m_q_KI= state.m_SimBodyStates[i].m_q.template tail<4>();
      bodies[i]->m_A_IK= getRotFromQuaternion<>(bodies[i]->m_q_KI);
    }
}


};

#endif

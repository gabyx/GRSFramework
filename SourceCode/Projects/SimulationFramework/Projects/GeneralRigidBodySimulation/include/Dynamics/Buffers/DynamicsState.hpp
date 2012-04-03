#ifndef DYNAMICS_STATE_HPP
#define DYNAMICS_STATE_HPP

#include <vector>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <boost/thread.hpp>

#include "AssertionDebug.hpp"
#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "QuaternionHelpers.hpp"

//Prototype
template< typename TLayoutConfig> class DynamicsState;
template< typename TLayoutConfig> class RigidBodyState;

/**
* @ingroup StatesAndBuffers
* @brief These are linear interpolation function to interpolate between two states!
* @{
*/
namespace Interpolate{

   template<typename TLayoutConfig>
   void lerp( const RigidBodyState<TLayoutConfig> & A, const RigidBodyState<TLayoutConfig> & B, RigidBodyState<TLayoutConfig> & X, typename TLayoutConfig::PREC factor){
      X.m_q = A.m_q + factor*(B.m_q - A.m_q);
      X.m_u = A.m_u + factor*(B.m_u - A.m_u);
   };

   template<typename TLayoutConfig>
   void lerp( const DynamicsState<TLayoutConfig> & A, const DynamicsState<TLayoutConfig> & B, DynamicsState<TLayoutConfig> & X, typename TLayoutConfig::PREC factor){
      ASSERTMSG(A.m_nSimBodies == B.m_nSimBodies &&  B.m_nSimBodies == X.m_nSimBodies ,"Wrong number of bodies!");
      X.m_t = X.m_t = A.m_t + factor*(B.m_t - A.m_t);
      for(int i=0;i<A.m_SimBodyStates.size();i++){
         lerp(A.m_SimBodyStates[i],B.m_SimBodyStates[i],X.m_SimBodyStates[i],factor);
      }
   };

};
/** @} */


/**
* @ingroup StatesAndBuffers
* @brief This represents a dynamic state of a rigid body.
*/
template< typename TLayoutConfig>
class RigidBodyState{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

      RigidBodyState(){
         m_u.setZero();
         m_q.setZero();
   };

   RigidBodyState<TLayoutConfig> & operator =(const RigidBodyState<TLayoutConfig>& state){
      m_u = state.m_u;
      m_q = state.m_q;
      return *this;
   }

   template<typename TLayoutConfig>
   friend void Interpolate::lerp( const RigidBodyState<TLayoutConfig> & A, const RigidBodyState<TLayoutConfig> & B, RigidBodyState<TLayoutConfig> & X, typename TLayoutConfig::PREC factor);

   Eigen::Matrix<PREC,NDOFuObj,1>	m_u; ///< These are the generalized velocities \f$\mathbf{u}\f$ of a rigid body.
   Eigen::Matrix<PREC,NDOFqObj,1>	m_q; ///< These are the generalized coordinates \f$\mathbf{q}\f$ of a rigid body.
};


/**
* @ingroup StatesAndBuffers
* @brief This represents a dynamic state of the dynamics system.
*/
template< typename TLayoutConfig>
class DynamicsState {
public:
   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      DynamicsState(const unsigned int nSimBodies);
   ~DynamicsState();

   DynamicsState<TLayoutConfig> & operator =(const DynamicsState<TLayoutConfig>& state);

   template<typename TLayoutConfig>
   friend void Interpolate::lerp( const DynamicsState<TLayoutConfig> & A, const DynamicsState<TLayoutConfig> & B, DynamicsState<TLayoutConfig> & X, typename TLayoutConfig::PREC factor);

   PREC	m_t; ///< The time in \f$[s]\f$
   std::vector< RigidBodyState<TLayoutConfig> >  m_SimBodyStates; ///< A vector comprising of all rigid body states of the system.

   const unsigned int m_nSimBodies;

   enum {NONE = 0, STARTSTATE=1, ENDSTATE = 2} m_StateType;
};



template<typename TLayoutConfig>
DynamicsState<TLayoutConfig>::DynamicsState(const unsigned int nSimBodies):
m_nSimBodies(nSimBodies)
{
   m_t = 0.0;
   m_StateType = NONE;
   m_SimBodyStates.assign(nSimBodies,RigidBodyState<TLayoutConfig>());
}

template<typename TLayoutConfig>
DynamicsState<TLayoutConfig>::~DynamicsState()
{
   //DECONSTRUCTOR_MESSAGE
}
template<typename TLayoutConfig>
DynamicsState<TLayoutConfig> & DynamicsState<TLayoutConfig>::operator =(const DynamicsState<TLayoutConfig>& state)
{
   m_StateType = state.m_StateType;
   m_t = state.m_t;
   ASSERTMSG( m_SimBodyStates.size() == state.m_SimBodyStates.size(), "DynamicsState:: Size mismatch!" );
   for(int i=0; i<state.m_SimBodyStates.size();i++){
      m_SimBodyStates[i] = state.m_SimBodyStates[i];
   }
   return *this;
}




#endif
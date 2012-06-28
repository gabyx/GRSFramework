﻿#ifndef InclusionSolverNT_hpp
#define InclusionSolverNT_hpp

#include <fstream>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "CollisionSolver.hpp"
#include "RigidBody.hpp"
#include "ContactParameterMap.hpp"
#include "PercussionPool.hpp"

#include "VectorToSkewMatrix.hpp"

#include "ProxFunctions.hpp"

#include "InclusionSolverSettings.hpp"

#include "LogDefines.hpp"

//#define USE_PERCUSSION_POOL 1

/**
* @ingroup Inclusion
* @brief The inclusion solver for an NT ordered problem.
*/
template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
class InclusionSolverNT{
public:
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InclusionSolverNT(boost::shared_ptr<TCollisionSolver >  pCollisionSolver, boost::shared_ptr<TDynamicsSystem> pDynSys);

  void initializeLog(Ogre::Log* pSolverLog);
  void reset();
  void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
  void solveInclusionProblem( const DynamicsState<TLayoutConfig> * state_s,
    const DynamicsState<TLayoutConfig> * state_m,
    DynamicsState<TLayoutConfig> * state_e);

  // These are updated from the System, over Inclusion Solver;
  VectorU m_Minv_diag;
  VectorU m_h;
  VectorU m_h_const;    // constant terms (gravity)
  VectorU m_delta_u_E;  // the delta which adds to the final u_E

  PREC m_G_conditionNumber;
  PREC m_G_notDiagDominant;
  unsigned int m_iterationsNeeded;
  bool m_bConverged;
  unsigned int m_nContacts;

  ContactParameterMap<TLayoutConfig> m_ContactParameterMap;

  PercussionPool<TLayoutConfig> m_PercussionPool;

  void reservePercussionPoolSpace(unsigned int nExpectedContacts);
  void readFromPercussionPool(unsigned int index, VectorDyn & P_Nold, VectorDyn & P_Told);
  void updatePercussionPool(const VectorDyn & P_Nold, const VectorDyn & P_Told ) ;


  InclusionSolverSettings<TLayoutConfig> m_Settings;

  unsigned int getNObjects();

protected:
  unsigned int m_nDofq, m_nDofu, m_nDofqObj, m_nDofuObj, m_nDofFriction, m_nSimBodies;

  unsigned int m_nExpectedContacts;

  boost::shared_ptr<TCollisionSolver> m_pCollisionSolver;
  boost::shared_ptr<TDynamicsSystem>  m_pDynSys;
  std::vector< boost::shared_ptr< RigidBody<TLayoutConfig> > > & m_SimBodies;
  std::vector< boost::shared_ptr< RigidBody<TLayoutConfig> > > & m_Bodies;



  // Matrices for solving the inclusion ===========================
  PREC m_nLambdas;
  MatrixUDyn m_W_N;
  MatrixUDyn m_W_T;

  VectorDyn m_xi_N;
  VectorDyn m_xi_T;

  // Helper Matrices, these can be computed efficiently!
  VectorDyn m_WN_uS;        // W_N.transpose() * m_u_S
  VectorDyn m_WT_uS;        // W_T.transpose() * m_u_S
  VectorDyn m_Minv_h_dt;    // M.inverse() * h * deltaT
  VectorDyn m_WN_Minv_h_dt; // W_N.transpose()* M.inverse() * h * deltaT
  VectorDyn m_WT_Minv_h_dt; // W_T.transpose()* M.inverse() * h * deltaT

  VectorDyn m_mu;
  VectorDyn m_I_epsilon_N;  // I + epsilon_N_diag
  VectorDyn m_I_epsilon_T;  // I + epsilon_T_diag

  // Two buffers to exchange fastly the percussions in the prox iterations
  VectorDyn* m_P_N_front;
  VectorDyn* m_P_N_back;
  VectorDyn* m_P_T_front;
  VectorDyn* m_P_T_back;
#define P_N_back (*m_P_N_back)
#define P_T_back (*m_P_T_back)
#define P_N_front (*m_P_N_front)
#define P_T_front (*m_P_T_front)
  void swapPercussionBuffer();
  void resetPercussionBuffer();
  VectorDyn m_P_N_1;
  VectorDyn m_P_T_1;
  VectorDyn m_P_N_2;
  VectorDyn m_P_T_2;
  // ==========================

  MatrixDynRow m_G_NN, m_G_NT;
  MatrixDynRow m_G_TT;

  VectorDyn m_c_N;
  VectorDyn m_c_T;

  VectorDyn m_R_N;
  VectorDyn m_R_T;
  // ==========================================================================

  inline void setupRMatrix(PREC alpha);
  inline void doJorProx();
  inline void doSorProx();




  // Log
  Ogre::Log*	m_pSolverLog;
  std::stringstream logstream;
};



template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
InclusionSolverNT<TLayoutConfig, TDynamicsSystem, TCollisionSolver>::InclusionSolverNT(boost::shared_ptr< TCollisionSolver >  pCollisionSolver,  boost::shared_ptr<TDynamicsSystem> pDynSys):
m_SimBodies(pCollisionSolver->m_SimBodies),
m_Bodies(pCollisionSolver->m_Bodies)
{

  m_nSimBodies = pCollisionSolver->m_nSimBodies;
  m_nDofqObj = NDOFqObj;
  m_nDofuObj = NDOFuObj;
  m_nDofq = m_nSimBodies * m_nDofqObj;
  m_nDofu = m_nSimBodies * m_nDofuObj;
  m_nDofFriction = NDOFFriction;

  m_Minv_diag.resize(m_nDofu);
  m_h.resize(m_nDofu);
  m_h_const.resize(m_nDofu);
  m_delta_u_E.resize(m_nDofu);
  m_Minv_h_dt.resize(m_nDofu);
  resetPercussionBuffer();

  m_pCollisionSolver = pCollisionSolver;
  m_nContacts = 0;

  m_iterationsNeeded =0;
  m_bConverged = false;
  m_G_conditionNumber = 0;
  m_G_notDiagDominant = 0;

  m_pDynSys = pDynSys;
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig, TDynamicsSystem, TCollisionSolver>::initializeLog( Ogre::Log* pSolverLog )
{
  m_pSolverLog = pSolverLog;
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
unsigned int InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::getNObjects()
{
  return m_nSimBodies;
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::reset()
{
  // Do a Debug check if sizes match!
  ASSERTMSG( m_SimBodies.size() * NDOFuObj == m_nDofu, "InclusionSolverNT:: Error in Dimension of System!");
  ASSERTMSG( m_SimBodies.size() * NDOFqObj == m_nDofq, "InclusionSolverNT:: Error in Dimension of System!");

  m_pDynSys->init_const_hTerm(m_h_const);
  m_pDynSys->init_MassMatrixInv(m_Minv_diag);


}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::resetForNextIter()
{
  m_h.setZero();
  m_nContacts = 0;

  m_iterationsNeeded =0;
  m_bConverged = false;
  m_G_conditionNumber = 0;
  m_G_notDiagDominant = 0;
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::swapPercussionBuffer()
{
  std::swap(m_P_N_back,m_P_N_front);
  std::swap(m_P_T_back,m_P_T_front);
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::resetPercussionBuffer()
{
  m_P_N_back = &m_P_N_1;
  m_P_T_back = &m_P_T_1;
  m_P_N_front = &m_P_N_2;
  m_P_T_front = &m_P_T_2;
}



template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig, TDynamicsSystem, TCollisionSolver>::reservePercussionPoolSpace( unsigned int nExpectedContacts )
{
   m_nExpectedContacts = nExpectedContacts;
   m_PercussionPool.rehashPercussionPool(m_nExpectedContacts);
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::solveInclusionProblem(const DynamicsState<TLayoutConfig> * state_s,
                                                                                        const DynamicsState<TLayoutConfig> * state_m,
                                                                                        DynamicsState<TLayoutConfig> * state_e)
{

#if CoutLevelSolver>0
  CLEARLOG;
  logstream << "---> solveInclusionProblem(): "<< std::endl;
  LOG(m_pSolverLog);
#endif


  // Add constant term to h
  m_h += m_h_const;

  // Update m_Minv_h_dt
  m_Minv_h_dt = m_Minv_diag.asDiagonal() * m_h * m_Settings.m_deltaT;
#if CoutLevelSolverWhenContact>2
  CLEARLOG;
  logstream << "h= "<< m_h.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
  logstream << "Minv_diag= "<< "diag(" <<m_Minv_diag.transpose().format(MyIOFormat::Matlab) <<");"<<std::endl;
  logstream << "Minv_h_dt= "<< m_Minv_h_dt.transpose().format(MyIOFormat::Matlab) <<"';"<<std::endl;
  LOG(m_pSolverLog);
#endif

  // Iterate over Collision set and assemble the matrices...
  std::vector<CollisionData<TLayoutConfig> > & collSet = m_pCollisionSolver->m_CollisionSet;
  m_nContacts = (unsigned int)collSet.size();
  m_iterationsNeeded = 0;
  m_bConverged = false;

  if(m_nContacts > 0){

    // Assign Space for matrices =====================================
    m_W_N.resize(m_nSimBodies * NDOFuObj ,  m_nContacts);
    m_W_T.resize(m_nSimBodies * NDOFuObj ,  m_nDofFriction*m_nContacts);
    m_xi_N.resize(m_nContacts);
    m_xi_T.resize(m_nDofFriction*m_nContacts);

    m_WN_uS.resize(m_nContacts);
    m_WT_uS.resize(m_nDofFriction*m_nContacts);
    m_WN_Minv_h_dt.resize(m_nContacts);
    m_WT_Minv_h_dt.resize(m_nDofFriction*m_nContacts);

    m_mu.resize(m_nContacts);
    m_I_epsilon_N.resize(m_nContacts);
    m_I_epsilon_T.resize(m_nDofFriction*m_nContacts);

    m_P_N_1.resize(m_nContacts);
    m_P_N_2.resize(m_nContacts);
    m_P_T_2.resize(m_nDofFriction*m_nContacts);
    m_P_T_1.resize(m_nDofFriction*m_nContacts);

    m_G_NT.resize(m_nContacts,m_nContacts);
    m_G_TT.resize(m_nContacts,m_nContacts);
    m_G_NN.resize(m_nContacts,m_nContacts);

    m_c_N.resize(m_nContacts);
    m_c_T.resize(m_nContacts);

    m_R_N.resize(m_nContacts);
    m_R_T.resize(m_nDofFriction*m_nContacts);
    // ==============================================================

    m_W_N.setZero();
    m_W_T.setZero();
    m_xi_N.setZero();
    m_xi_T.setZero();

    resetPercussionBuffer();
    P_N_back.setZero();
    P_T_back.setZero();

    m_WN_Minv_h_dt.setZero();
    m_WT_Minv_h_dt.setZero();
    m_WN_uS.setZero();
    m_WT_uS.setZero();


    // Assemble W_N and W_T and xi_N and xi_T =====================================================
    static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
    static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
    static VectorUObj w_N_part, w_T_part;
    for ( unsigned int contactIdx = 0 ; contactIdx < m_nContacts ; contactIdx++)
    {

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "e_x= "<< collSet[contactIdx].m_e_x.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      logstream << "e_y= "<< collSet[contactIdx].m_e_y.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      logstream << "e_z= "<< collSet[contactIdx].m_e_z.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      logstream << collSet[contactIdx].m_e_x.dot(collSet[contactIdx].m_e_y) << collSet[contactIdx].m_e_y.dot(collSet[contactIdx].m_e_z) <<std::endl;
      logstream << "r_s1c1="<< collSet[contactIdx].m_r_S1C1.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      LOG(m_pSolverLog);
#endif
      ASSERTMSG( std::abs(collSet[contactIdx].m_e_x.dot(collSet[contactIdx].m_e_y)) < 1e-3 && std::abs(collSet[contactIdx].m_e_y.dot(collSet[contactIdx].m_e_z))< 1e-3, "Vectors not parallel");



      int id1 = collSet[contactIdx].m_pBody1->m_id;
      int id2 = collSet[contactIdx].m_pBody2->m_id;

      // Fill the entries for Body 1 =================================================
      if( collSet[contactIdx].m_pBody1->m_eState == RigidBody<TLayoutConfig>::SIMULATED ){

        // Contact goes into W_N, W_T
        updateSkewSymmetricMatrix<>( collSet[contactIdx].m_r_S1C1, I_r_SiCi_hat);
        I_Jacobi_2 = ( collSet[contactIdx].m_pBody1->m_A_IK.transpose() * I_r_SiCi_hat );

        // N direction =================================================
        w_N_part.template head<3>() = - collSet[contactIdx].m_e_z; // I frame
        w_N_part.template tail<3>() = - I_Jacobi_2 * collSet[contactIdx].m_e_z;
        // pop into W_N
        m_W_N.template block<NDOFuObj,1>( id1 * NDOFuObj ,  contactIdx) =  w_N_part;

        // compute part to m_WN_uS and to m_WN_Minv_h_dt
        m_WN_uS(contactIdx)        += w_N_part.dot( state_s->m_SimBodyStates[id1].m_u );
        m_WN_Minv_h_dt(contactIdx) += w_N_part.dot( m_Minv_h_dt.template segment<NDOFuObj>( id1 * NDOFuObj ));

        // T1 direction =================================================
        w_T_part.template head<3>() = - collSet[contactIdx].m_e_x; // I frame
        w_T_part.template tail<3>() = - I_Jacobi_2 * collSet[contactIdx].m_e_x;
        // pop into W_T
        m_W_T.template block<NDOFuObj,1>( id1 * NDOFuObj ,  m_nDofFriction*contactIdx) =  w_T_part ;
        // compute part to m_WT_uS and to m_WT_Minv_h_dt
        m_WT_uS(m_nDofFriction*contactIdx)        += w_T_part.dot( state_s->m_SimBodyStates[id1].m_u );
        m_WT_Minv_h_dt(m_nDofFriction*contactIdx) += w_T_part.dot( m_Minv_h_dt.template segment<NDOFuObj>( id1 * NDOFuObj ));

        // T2 direction =================================================
        w_T_part.template head<3>() = - collSet[contactIdx].m_e_y; // I frame
        w_T_part.template tail<3>() = - I_Jacobi_2 * collSet[contactIdx].m_e_y;
        // pop into W_T
        m_W_T.template block<NDOFuObj,1>( id1 * NDOFuObj ,  m_nDofFriction*contactIdx + 1) = w_T_part;
        // compute part to m_WT_uS
        m_WT_uS(m_nDofFriction*contactIdx + 1)        += w_T_part.dot( state_s->m_SimBodyStates[id1].m_u );
        m_WT_Minv_h_dt(m_nDofFriction*contactIdx +1)  += w_T_part.dot( m_Minv_h_dt.template segment<NDOFuObj>( id1 * NDOFuObj ) );

      }
      else if( collSet[contactIdx].m_pBody1->m_eState == RigidBody<TLayoutConfig>::ANIMATED ){
        // Contact goes into xi_N, xi_T

      }


      // Fill the entries for Body 2 =================================================
      if( collSet[contactIdx].m_pBody2->m_eState == RigidBody<TLayoutConfig>::SIMULATED ){

        // Contact goes into W_N, W_T
        updateSkewSymmetricMatrix<>( collSet[contactIdx].m_r_S2C2, I_r_SiCi_hat);
        I_Jacobi_2 = ( collSet[contactIdx].m_pBody2->m_A_IK.transpose() * I_r_SiCi_hat );

        // N direction =================================================
        w_N_part.template head<3>() =  collSet[contactIdx].m_e_z;
        w_N_part.template tail<3>() =  I_Jacobi_2 * collSet[contactIdx].m_e_z;
        // pop into W_N
        m_W_N.template block<NDOFuObj,1>( id2 * NDOFuObj ,  contactIdx) =  w_N_part; // I frame
        // compute part to m_WN_uS and to m_WN_Minv_h_dt
        m_WN_uS(contactIdx)         += w_N_part.dot( state_s->m_SimBodyStates[id2].m_u );
        m_WN_Minv_h_dt(contactIdx)  += w_N_part.dot( m_Minv_h_dt.template segment<NDOFuObj>( id2 * NDOFuObj ));

        // T1 direction =================================================
        w_T_part.template head<3>() =  collSet[contactIdx].m_e_x;
        w_T_part.template tail<3>() =  I_Jacobi_2 * collSet[contactIdx].m_e_x;
        // pop into W_T
        m_W_T.template block<NDOFuObj,1>( id2 * NDOFuObj ,  m_nDofFriction*contactIdx) =  w_T_part ; // I frame
        // compute part to m_WT_uS and to m_WT_Minv_h_dt
        m_WT_uS(m_nDofFriction*contactIdx)        += w_T_part.dot( state_s->m_SimBodyStates[id2].m_u );
        m_WT_Minv_h_dt(m_nDofFriction*contactIdx) += w_T_part.dot( m_Minv_h_dt.template segment<NDOFuObj>( id2 * NDOFuObj ));

        // T2 direction =================================================
        w_T_part.template head<3>() =  collSet[contactIdx].m_e_y;
        w_T_part.template tail<3>() =  I_Jacobi_2 * collSet[contactIdx].m_e_y;
        // pop into W_T
        m_W_T.template block<NDOFuObj,1>( id2 * NDOFuObj ,  m_nDofFriction*contactIdx + 1) = w_T_part; // I frame
        // compute part to m_WT_uS and to m_WT_Minv_h_dt
        m_WT_uS(m_nDofFriction*contactIdx + 1)        += w_T_part.dot( state_s->m_SimBodyStates[id2].m_u );
        m_WT_Minv_h_dt(m_nDofFriction*contactIdx + 1) += w_T_part.dot( m_Minv_h_dt.template segment<NDOFuObj>( id2 * NDOFuObj ));

      }
      else if( collSet[contactIdx].m_pBody1->m_eState == RigidBody<TLayoutConfig>::ANIMATED ){
        // Contact goes into xi_N, xi_T
      }

      // Get the ContactParams and fill the parameters
      ContactParams<TLayoutConfig> & params  = m_ContactParameterMap.getContactParams(collSet[contactIdx].m_pBody1->m_eMaterial,collSet[contactIdx].m_pBody2->m_eMaterial);
      m_mu(contactIdx)                              = params.m_mu;
      m_I_epsilon_N(contactIdx)                     = 1 + params.m_epsilon_N;
      m_I_epsilon_T(m_nDofFriction*contactIdx)      = 1 + params.m_epsilon_T;
      m_I_epsilon_T(m_nDofFriction*contactIdx + 1)  = 1 + params.m_epsilon_T;

      // Fill in Percussions
#if USE_PERCUSSION_POOL == 1
      readFromPercussionPool(contactIdx,P_N_back,P_T_back);
#endif

    }
    // =============================================================================================================

#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "W_N= ..."<< std::endl << m_W_N.format(MyIOFormat::Matlab)<<";"<<std::endl;
    logstream << "W_T= ..."<< std::endl << m_W_T.format(MyIOFormat::Matlab)<<";"<<std::endl;
    logstream << "mu= "<<"diag("<< m_mu.transpose().format(MyIOFormat::Matlab)<<");"<<std::endl;
    logstream << "I_epsilon_N= " << "diag("<< m_I_epsilon_N.transpose().format(MyIOFormat::Matlab)<<");"<<std::endl;
    logstream << "I_epsilon_T= " << "diag("<< m_I_epsilon_T.transpose().format(MyIOFormat::Matlab)<<");"<<std::endl;
    logstream << "WT_uS= "<< m_WN_uS.transpose().format(MyIOFormat::Matlab)<<std::endl;
    logstream << "WN_uS= "<< m_WT_uS.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    logstream << "WN_Minv_h_dt= "<< m_WN_Minv_h_dt.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    logstream << "WT_Minv_h_dt= "<< m_WT_Minv_h_dt.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    LOG(m_pSolverLog);
#endif

    m_G_NN.noalias() = m_W_N.transpose() * m_Minv_diag.asDiagonal() * m_W_N;
    MatrixUDyn tempMinv_WT;
    tempMinv_WT.noalias() = m_Minv_diag.asDiagonal() * m_W_T;
    m_G_NT.noalias() = m_W_N.transpose() * tempMinv_WT;
    m_G_TT.noalias() = m_W_T.transpose() * tempMinv_WT;

#if CALCULATE_COND_OF_G == 1 || CALCULATE_DIAGDOM_OF_G == 1
    MatrixDyn G(m_nContacts*3,m_nContacts*3);
    G.block(0,0,m_G_NN.rows(),m_G_NN.cols()).noalias() = m_G_NN;
    G.block(0,m_G_NN.cols(),m_G_NT.rows(),m_G_NT.cols()).noalias() = m_G_NT;
    G.block(m_G_NN.rows(),0,m_G_NT.transpose().rows(),m_G_NT.transpose().cols()).noalias() = m_G_NT.transpose();
    G.block(m_G_NN.rows(),m_G_NN.cols(),m_G_TT.rows(),m_G_TT.cols()).noalias() = m_G_TT;
#if CALCULATE_COND_OF_G == 1
      JacobiSVD<MatrixXd> svd(G);
      m_G_conditionNumber = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
#endif
#if CALCULATE_DIAGDOM_OF_G == 1
      m_G_notDiagDominant = (G.diagonal().array() < (G.rowwise().sum()-G.diagonal()).array()).sum();
#endif
#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "G= ..."<<std::endl<< G.format(MyIOFormat::Matlab)<<";"<<std::endl;
    LOG(m_pSolverLog);
#endif
#endif





#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "G_NN= ..."<< std::endl << m_G_NN.format(MyIOFormat::Matlab)<<";"<<std::endl;
    logstream << "G_NT= ..."<< std::endl << m_G_NT.format(MyIOFormat::Matlab)<<";"<<std::endl;
    logstream << "G_TT= ..."<< std::endl << m_G_TT.format(MyIOFormat::Matlab)<<";"<<std::endl;
    LOG(m_pSolverLog);
#endif

    m_c_N.noalias() = m_WN_Minv_h_dt + m_I_epsilon_N.asDiagonal() * ( m_WN_uS  + m_xi_N);
    m_c_T.noalias() = m_WT_Minv_h_dt + m_I_epsilon_T.asDiagonal() * ( m_WT_uS  + m_xi_T);

#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "c_N= " << m_c_N.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    logstream << "c_T= " << m_c_T.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    LOG(m_pSolverLog);
#endif



#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "P_N_back= "<<P_N_back.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    logstream << "P_T_back= "<<P_T_back.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
    LOG(m_pSolverLog);
#endif


    //doSorProx();
    doJorProx();

    //TODO update ContactPercussions
#if USE_PERCUSSION_POOL == 1
    updatePercussionPool(P_N_front, P_T_front);
#endif

#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "% Prox Iterations needed: "<< m_iterationsNeeded <<std::endl;
    LOG(m_pSolverLog);
#endif

    // Calculate u_E for each body in the state...
    m_delta_u_E.noalias() = m_Minv_h_dt + m_Minv_diag.asDiagonal() * (m_W_N * P_N_front + m_W_T * P_T_front);

    for(unsigned int i=0; i < m_nSimBodies; i++){
      state_e->m_SimBodyStates[i].m_u = state_s->m_SimBodyStates[i].m_u + m_delta_u_E.template segment<NDOFuObj>(i*NDOFuObj);
    }



  }
  else{
    // Do simple timestep to u_E for each body in the state...

    for(unsigned int i=0; i < m_nSimBodies; i++){
      state_e->m_SimBodyStates[i].m_u = state_s->m_SimBodyStates[i].m_u + m_Minv_h_dt.template segment<NDOFuObj>(i*NDOFuObj);
    }
  }

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::setupRMatrix(PREC alpha){
 // Calculate  R_N, R_T,
   PREC r_T_i ;
   m_R_N.noalias() =  m_G_NN.diagonal().array().inverse().matrix() * m_Settings.m_alphaSORProx;
   for(unsigned int i=0;i<m_nContacts;i++){
      r_T_i = m_Settings.m_alphaSORProx / (m_G_TT.diagonal().template segment<NDOFFriction>(m_nDofFriction*i)).maxCoeff();
      m_R_T(m_nDofFriction*i) = r_T_i;
      m_R_T(m_nDofFriction*i+1) = r_T_i;
   }


#if CoutLevelSolverWhenContact>2
    CLEARLOG;
    logstream << "R_N= "<< "diag(" << m_R_N.transpose().format(MyIOFormat::Matlab)<<"');"<<std::endl;
    logstream << "R_T= "<< "diag(" << m_R_T.transpose().format(MyIOFormat::Matlab)<<"');"<<std::endl;
    LOG(m_pSolverLog);
#endif

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::doJorProx(){

   // Calculate  R_N, R_T,
   setupRMatrix(m_Settings.m_alphaJORProx);

   // Prox- Iteration
   while (m_iterationsNeeded < m_Settings.m_MaxIter)
   {
      P_N_front.noalias() = P_N_back - m_R_N.asDiagonal() * (m_G_NN * P_N_back + m_G_NT * P_T_back + m_c_N);
      Prox::ProxFunction<ConvexSets::RPlus>::doProxMulti(P_N_front);

      ////Solve Prox on Tangential direction
      P_T_front = P_T_back - m_R_T.asDiagonal() * (m_G_NT.transpose()*P_N_front + m_G_TT*P_T_back + m_c_T);
      Prox::ProxFunction<ConvexSets::Disk>::doProxMulti(m_mu.asDiagonal() * P_N_front, P_T_front);

      //Calculate CancelCriteria
      m_bConverged = Numerics::cancelCriteriaValue(P_N_back,P_N_front,P_T_back,P_T_front, m_Settings.m_AbsTol, m_Settings.m_RelTol);

   #if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "P_N_front= "<<P_N_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      logstream << "P_T_front= "<<P_T_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      LOG(m_pSolverLog);
   #endif

      m_iterationsNeeded++;

      if (m_bConverged == true)
      {
         break;
      }
      swapPercussionBuffer();
   }

}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::doSorProx(){

   setupRMatrix(m_Settings.m_alphaSORProx);

   // Prox- Iteration
   while (m_iterationsNeeded < m_Settings.m_MaxIter)
   {

      // Copy all values
      P_N_front.noalias() = P_N_back;
      P_T_front.noalias() = P_T_back;

      for(unsigned int i=0; i < m_nContacts; i++){

         // Iterate SOR scheme only with P_front!
         // Solve Prox on Normal direction
         P_N_front(i) = P_N_front(i) - m_R_N(i) * (m_G_NN.row(i).dot(P_N_front) + m_G_NT.row(i).dot(P_T_front) + m_c_N(i) );
         Prox::ProxFunction<ConvexSets::RPlus>::doProxSingle( P_N_front(i) );

         // Solve Prox on Tangential direction
        P_T_front.template segment<NDOFFriction>(NDOFFriction*i) =
         P_T_front.template segment<NDOFFriction>(NDOFFriction*i) - (m_R_T.template segment<NDOFFriction>(NDOFFriction*i)).asDiagonal()
          * (m_G_NT.transpose().block(NDOFFriction*i,0,NDOFFriction,m_nContacts) * P_N_front
           +m_G_TT.block(NDOFFriction*i,0,NDOFFriction,NDOFFriction*m_nContacts) * P_T_front
           + m_c_T.segment<NDOFFriction>(NDOFFriction*i));


         Prox::ProxFunction<ConvexSets::Disk>::doProxSingle(m_mu(i) * P_N_front(i), P_T_front.template segment<NDOFFriction>(NDOFFriction*i));

      }


      //Calculate CancelCriteria
      m_bConverged = Numerics::cancelCriteriaValue(P_N_back,P_N_front,P_T_back,P_T_front, m_Settings.m_AbsTol, m_Settings.m_RelTol);

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "P_N_front= "<<P_N_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      logstream << "P_T_front= "<<P_T_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;
      LOG(m_pSolverLog);
#endif

      m_iterationsNeeded++;

      if (m_bConverged == true)
      {
         break;
      }
      swapPercussionBuffer();
   }
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void InclusionSolverNT<TLayoutConfig,TDynamicsSystem, TCollisionSolver>::updatePercussionPool(const VectorDyn & P_Nold, const VectorDyn & P_Told )
{
   static VectorPContact P_contact;
    for(unsigned int i = 0; i< m_nContacts; i++){
       P_contact(0) = P_Nold(i);
       P_contact(1) = P_Told(NDOFFriction*i);
       P_contact(2) = P_Told(NDOFFriction*i+1);
       m_PercussionPool.setPercussion(i,P_contact);
    }

    m_PercussionPool.clearUsedPercussionList();
    m_PercussionPool.removeUnusedPercussions();
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver >
void  InclusionSolverNT<TLayoutConfig, TDynamicsSystem, TCollisionSolver>::readFromPercussionPool(unsigned int index, VectorDyn & P_Nold, VectorDyn & P_Told)
{
   static VectorPContact P_contact;
   m_PercussionPool.getPercussion(m_pCollisionSolver->m_CollisionSet[index].m_ContactTag,P_contact);
   P_Nold(index) = P_contact(0);
   P_Told(NDOFFriction*index) =   P_contact(1);
   P_Told(NDOFFriction*index+1) = P_contact(2);
}



#endif

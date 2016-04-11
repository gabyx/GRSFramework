// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_InclusionSolverNTContactOrdered_hpp
#define GRSF_dynamics_inclusion_InclusionSolverNTContactOrdered_hpp

#include <fstream>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/collision/CollisionSolver.hpp"
#include "GRSF/dynamics/general/RigidBody.hpp"

#include "GRSF/dynamics/inclusion/PercussionPool.hpp"

#include "GRSF/dynamics/general/VectorToSkewMatrix.hpp"

#include "GRSF/dynamics/inclusion/ProxFunctions.hpp"

#include "GRSF/dynamics/inclusion/InclusionSolverSettings.hpp"

#include "GRSF/common/LogDefines.hpp"

#ifndef USE_PERCUSSION_POOL
#define USE_PERCUSSION_POOL 0
#endif

/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem.
*/
template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
class InclusionSolverNTContactOrdered{
public:
   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      InclusionSolverNTContactOrdered(std::shared_ptr<TCollisionSolver >  pCollisionSolver, std::shared_ptr<TDynamicsSystem> pDynSys);

   void initializeLog(Ogre::Log* pSolverLog);
   void reset();
   void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
   void solveInclusionProblem( const DynamicsState<TLayoutConfig> * state_s, const DynamicsState<TLayoutConfig> * state_m, DynamicsState<TLayoutConfig> * state_e);

   // These are updated from the System, over Inclusion Solver;
   //VectorU m_Minv_diag;
   //VectorU m_h;
   //VectorU m_h_const;    // constant terms (gravity)
   //VectorU m_delta_u_E;  // the delta which adds to the final u_E

   PREC m_G_conditionNumber;
   unsigned int m_iterationsNeeded;
   bool m_bConverged;
   unsigned int m_nContacts;

   ContactParameterMap<TLayoutConfig> m_ContactParameterMap;

   PercussionPool<TLayoutConfig> m_PercussionPool;

   void reservePercussionPoolSpace(unsigned int nExpectedContacts);
   void readFromPercussionPool(unsigned int index, const CollisionData<TLayoutConfig> * pCollData, VectorDyn & P_old);
   void updatePercussionPool(const VectorDyn & P_old ) ;


   InclusionSolverSettings<TLayoutConfig> m_Settings;

   unsigned int getNObjects();

protected:
   unsigned int m_nDofq, m_nDofu, m_nDofqObj, m_nDofuObj, m_nDofFriction, m_nSimBodies;

   unsigned int m_nExpectedContacts;

   std::shared_ptr<TCollisionSolver> m_pCollisionSolver;
   std::shared_ptr<TDynamicsSystem>  m_pDynSys;
   std::vector< std::shared_ptr< RigidBody<TLayoutConfig> > > & m_SimBodies;
   std::vector< std::shared_ptr< RigidBody<TLayoutConfig> > > & m_Bodies;

   TContactGraph m_ContactGraph;

   // Matrices for solving the inclusion ===========================
   PREC m_nLambdas;
   //MatrixUDyn m_W;
   //VectorDyn m_xi;

   // Helper Matrices, these can be computed efficiently!
   //VectorDyn m_W_uS;        // W.transpose() * m_u_S
   //VectorDyn m_Minv_h_dt;   // M.inverse() * h * deltaT
   //VectorDyn m_W_Minv_h_dt; // W.transpose()* M.inverse() * h * deltaT

   VectorDyn m_mu;
   //VectorDyn m_I_epsilon;  // I + epsilon_diag

   // Two buffers to exchange fastly the percussions in the prox iterations
   VectorDyn* m_P_front;
   VectorDyn* m_P_back;

#define P_back (*m_P_back)
#define P_front (*m_P_front)

   void swapPercussionBuffer();
   void resetPercussionBuffer();
   VectorDyn m_P_1;
   VectorDyn m_P_2;
   // ==========================

   MatrixDynRow m_G;

   VectorDyn m_c;

   VectorDyn m_R;
   // ==========================================================================

   inline void setupRMatrix(PREC alpha);
   inline void doJorProx();
   inline void doSorProx();




   // Log
   Ogre::Log*	m_pSolverLog;
   std::stringstream logstream;
};



template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::InclusionSolverNTContactOrdered(std::shared_ptr< TCollisionSolver >  pCollisionSolver,  std::shared_ptr<TDynamicsSystem> pDynSys):
m_SimBodies(pCollisionSolver->m_SimBodies),
   m_Bodies(pCollisionSolver->m_Bodies)
{

   m_nSimBodies = pCollisionSolver->m_nSimBodies;
   m_nDofqObj = NDOFqObj;
   m_nDofuObj = NDOFuObj;
   m_nDofq = m_nSimBodies * m_nDofqObj;
   m_nDofu = m_nSimBodies * m_nDofuObj;
   m_nDofFriction = NDOFFriction;

   //m_delta_u_E.resize(m_nDofu);

   resetPercussionBuffer();

   m_pCollisionSolver = pCollisionSolver;
   //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
   m_pCollisionSolver->m_ContactDelegateList.addContactDelegate( 
      ContactDelegateList<TLayoutConfig>::ContactDelegate::from_method< TContactGraph,  &TContactGraph::addNode>(&m_ContactGraph) 
     );

   m_nContacts = 0;
   m_nLambdas = 0;

   m_iterationsNeeded =0;
   m_bConverged = false;
   m_G_conditionNumber = 0;

   m_pDynSys = pDynSys;
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::initializeLog( Ogre::Log* pSolverLog )
{
   m_pSolverLog = pSolverLog;
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
unsigned int InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::getNObjects()
{
   return m_nSimBodies;
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::reset()
{
   // Do a Debug check if sizes match!
   GRSF_ASSERTMSG( m_SimBodies.size() * NDOFuObj == m_nDofu, "InclusionSolverNTContactOrdered:: Error in Dimension of System!");
   GRSF_ASSERTMSG( m_SimBodies.size() * NDOFqObj == m_nDofq, "InclusionSolverNTContactOrdered:: Error in Dimension of System!");

   m_pDynSys->init_const_hTerm();
   m_pDynSys->init_const_MassMatrixInv();


}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::resetForNextIter()
{
   //m_h.setZero();
   m_ContactGraph.clearGraph();
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::swapPercussionBuffer()
{
   std::swap(m_P_back,m_P_front);
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::resetPercussionBuffer()
{
   m_P_back = &m_P_1;
   m_P_front = &m_P_2;
}



template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::reservePercussionPoolSpace( unsigned int nExpectedContacts )
{
   m_nExpectedContacts = nExpectedContacts;
   m_PercussionPool.rehashPercussionPool(m_nExpectedContacts);
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::solveInclusionProblem(const DynamicsState<TLayoutConfig> * state_s,
   const DynamicsState<TLayoutConfig> * state_m,
   DynamicsState<TLayoutConfig> * state_e)
{

#if CoutLevelSolver>0
   CLEARLOG;
   logstream << "---> solveInclusionProblem(): "<< endl;
   LOG(m_pSolverLog);
#endif

   // Iterate over all nodes set and assemble the matrices...
  TContactGraph::NodeList & nodes = m_ContactGraph.getNodeListRef();
  TContactGraph::Node * currentContactNode; 
   m_nContacts = (unsigned int)nodes.size();
   m_iterationsNeeded = 0;
   m_bConverged = false;

   if(m_nContacts > 0){


      m_nLambdas = m_ContactGraph.m_nLambdas;

      // Assign Space for matrices =====================================
      m_mu.resize(m_ContactGraph.m_nFrictionParams);

      m_P_1.resize(m_nLambdas);
      m_P_2.resize(m_nLambdas);

      m_G.setZero(m_nLambdas,m_nLambdas);

      m_c.setZero(m_nLambdas);

      m_R.resize(m_nLambdas);
      // ==============================================================

      resetPercussionBuffer();
      P_back.setZero();


      // Assemble W_N and W_T and xi_N and xi_T =====================================================
      static const CollisionData<TLayoutConfig> * pCollData;
      static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
      static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
      static VectorUObj w_part;

      static Eigen::Matrix<PREC,Eigen::Dynamic,Eigen::Dynamic> G_part;
      static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic>* W_j_body; 
      static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic>* W_i_body;
      static Eigen::Matrix<PREC,Eigen::Dynamic,NDOFuObj> W_i_bodyT_M_body;

      for ( unsigned int contactIdx = 0 ; contactIdx < m_nContacts ; contactIdx++)
      {

         currentContactNode = nodes[contactIdx];

         unsigned int i = currentContactNode->m_nodeNumber; // the current node number 
         unsigned int j;                                    // the current node number of the incident node!

         pCollData = currentContactNode->m_nodeData.m_pCollData;

         // Write mu parameters to m_mu
         m_mu(i) = currentContactNode->m_nodeData.m_mu(0);


         // iterate over all edges in current contact to build up G;
         TContactGraph::EdgeListIterator it;
         RigidBody<TLayoutConfig> * edgesBody;
         
         for(it = currentContactNode->m_edgeList.begin(); it != currentContactNode->m_edgeList.end(); it++){
            
            edgesBody = (*it)->m_edgeData.m_pBody;
            unsigned int bodyId = edgesBody->m_id;


             // We skip the edges which ar pointing towards the current node, we add them anyway, and also the symetric part!
            if( (*it)->m_startNode->m_nodeNumber  !=  i ) {
               continue;
            }


            j = (*it)->m_endNode->m_nodeNumber;
            W_j_body = TContactGraph::getW_body((*it)->m_endNode->m_nodeData, edgesBody);

            
            // Compute G 
            // Calculate G (Start = pNode (i) , End= (*it) (j) )

            W_i_body = TContactGraph::getW_body(currentContactNode->m_nodeData,edgesBody);
            

            if(i == j){ // We are on a self referencing edge! Thats good build diagonal of G and parts of c
               W_i_bodyT_M_body = (*W_i_body).transpose() * edgesBody->m_MassMatrixInv_diag.asDiagonal();
               m_G.block<(NDOFFriction+1),(NDOFFriction+1)>((NDOFFriction+1)*i,(NDOFFriction+1)*j).noalias()  += W_i_bodyT_M_body * (*W_i_body);
               m_c.segment<NDOFFriction+1>((NDOFFriction+1)*i).noalias() +=  W_i_bodyT_M_body * edgesBody->m_h_term * m_Settings.m_Timestep + 
                                                                             currentContactNode->m_nodeData.m_I_plus_eps.asDiagonal()*( (*W_i_body).transpose() * state_s->m_SimBodyStates[bodyId].m_u );
            }                                                                
            else{
              G_part = (*W_i_body).transpose() * edgesBody->m_MassMatrixInv_diag.asDiagonal() * (*W_j_body);
              m_G.block<(NDOFFriction+1),(NDOFFriction+1)>((NDOFFriction+1)*i,(NDOFFriction+1)*j).noalias() = G_part;
              m_G.block<(NDOFFriction+1),(NDOFFriction+1)>((NDOFFriction+1)*j,(NDOFFriction+1)*i).noalias() = G_part;
            }

         }
 
         // add once xi to c
         m_c.segment<NDOFFriction+1>((NDOFFriction+1)*i).noalias() +=  currentContactNode->m_nodeData.m_I_plus_eps.asDiagonal() * currentContactNode->m_nodeData.m_xi;

         
         // Fill in Percussions
#if USE_PERCUSSION_POOL == 1
         readFromPercussionPool(contactIdx,pCollData,P_back);
#endif

      }
      // =============================================================================================================


#if CALCULATE_COND_OF_G == 1
      JacobiSVD<MatrixXd> svd(m_G);
      m_G_conditionNumber = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
#endif

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "G= ..."<< endl << m_G.format(MyIOFormat::Matlab)<<";"<<endl;
      LOG(m_pSolverLog);
#endif

     // m_c.noalias() = m_W_Minv_h_dt + m_I_epsilon.asDiagonal() * ( m_W_uS  + m_xi);

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "c= " << m_c.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
      LOG(m_pSolverLog);
#endif

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "P_back= "<<P_back.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
      LOG(m_pSolverLog);
#endif
       
      //doSorProx();

      doJorProx();

      //TODO update ContactPercussions
#if USE_PERCUSSION_POOL == 1
      updatePercussionPool(P_front);
#endif

cout << "% Prox Iterations needed: "<< m_iterationsNeeded <<endl;
#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "% Prox Iterations needed: "<< m_iterationsNeeded <<endl;
      LOG(m_pSolverLog);
#endif

      // Calculate u_E for each body in the state...
      /*m_delta_u_E.noalias() = m_Minv_h_dt + m_Minv_diag.asDiagonal() * (m_W * P_front);*/

      static Eigen::Matrix<PREC,NDOFuObj,1> delta_u_E;
      for(unsigned int i=0; i < m_nSimBodies; i++){
         delta_u_E = m_SimBodies[i]->m_h_term * m_Settings.m_Timestep;

         TContactGraph::BodyToContactsListIterator itList  = m_ContactGraph.m_BodyToContactsList.find(m_SimBodies[i].get());
         // itList->second is the NodeList!
         if(itList != m_ContactGraph.m_BodyToContactsList.end()){
            for(TContactGraph::NodeListIterator it = itList->second.begin(); it != itList->second.end(); it++){
               delta_u_E.noalias() += *(TContactGraph::getW_body((*it)->m_nodeData,m_SimBodies[i].get())) * P_front.segment<NDOFFriction+1>( (*it)->m_nodeNumber * (NDOFFriction+1));
            }
         }
            
         state_e->m_SimBodyStates[i].m_u = state_s->m_SimBodyStates[i].m_u + m_SimBodies[i]->m_MassMatrixInv_diag.asDiagonal()*(delta_u_E);
         
      }
   }
   else{
      // Do simple timestep to u_E for each body in the state...

      for(unsigned int i=0; i < m_nSimBodies; i++){
         state_e->m_SimBodyStates[i].m_u = state_s->m_SimBodyStates[i].m_u + m_SimBodies[i]->m_MassMatrixInv_diag.asDiagonal()*m_SimBodies[i]->m_h_term * m_Settings.m_Timestep;;
      }
   }

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::setupRMatrix(PREC alpha){
   PREC r_T_i;
   for(unsigned int i=0;i<m_nContacts;i++){
      m_R((NDOFFriction+1)*i) =  alpha / m_G((NDOFFriction+1)*i,(NDOFFriction+1)*i);
      r_T_i = alpha / (m_G.diagonal().segment<NDOFFriction>((NDOFFriction+1)*i+1)).maxCoeff();
      m_R((NDOFFriction+1)*i+1) = r_T_i;
      m_R((NDOFFriction+1)*i+2) = r_T_i;
   }


   #if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "R= "<< "diag(" << m_R.transpose().format(MyIOFormat::Matlab)<<"');"<<endl;
      LOG(m_pSolverLog);
   #endif

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::doJorProx(){

   // This JOR Prox is worse then JOR prox of InclusionSolverNT which uses P_Front_N already for tangential directions.

   // Calculate  R_N, R_T,
   setupRMatrix(m_Settings.m_alphaJORProx);

   // Prox- Iteration
   while (m_iterationsNeeded < m_Settings.m_MaxIter)
   {

      P_front.noalias() = P_back - m_R.asDiagonal()*(m_G *P_back + m_c);
      Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxMulti(m_mu,P_front);

      //Calculate CancelCriteria
      m_bConverged = Numerics::cancelCriteriaValue(P_back,P_front, m_Settings.m_AbsTol, m_Settings.m_RelTol);

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
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

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::doSorProx(){

   static VectorPContact PContact_back;
   static unsigned int counterConverged;
   // Calculate  R_N, R_T,
   setupRMatrix(m_Settings.m_alphaSORProx);

   // P_back is filled with initial values, swap so P_front is filled with initial values, and only work with P_front;
   swapPercussionBuffer();

   // Prox- Iteration
   while (m_iterationsNeeded < m_Settings.m_MaxIter)
   {
      m_bConverged = false;
      counterConverged = 0;

      // Prox all, with sequential Sor Style..
      for(unsigned int i=0; i < m_nContacts; i++){

         // Prox the contact
         //Eigen::VectorBlock<VectorDyn,3> &PContact_front = P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i);
         PContact_back = P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i) ; // Save last values

         // This statement looks like aliasing but it does not! All Matrices*Matrices or MAtrices*Vector get evaluated into temporary by default
         // TODO Check if there is a Aliasing effect
         P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i) =
               P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i) -
                  (m_R.segment<NDOFFriction+1>((NDOFFriction+1)*i)).asDiagonal()*
                     (
                        (m_G.block((NDOFFriction+1)*i,0,NDOFFriction+1,m_nLambdas) * P_front) + m_c.segment<NDOFFriction+1>((NDOFFriction+1)*i)
                     );

         Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(m_mu(i),P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i));
         Numerics::cancelCriteriaValue(PContact_back, P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i), m_Settings.m_AbsTol, m_Settings.m_RelTol, counterConverged);

      }

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << "P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
      LOG(m_pSolverLog);
#endif

      m_iterationsNeeded++;

      if (counterConverged == m_nContacts)
      {
         m_bConverged = true;
         // P_front has newest values.
         break;
      }
   }
  
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::updatePercussionPool(const VectorDyn & P_old )
{
   static VectorPContact P_contact;
   for(unsigned int i = 0; i< m_nContacts; i++){
      P_contact(0) = P_old(i);
      P_contact(1) = P_old(NDOFFriction*i);
      P_contact(2) = P_old(NDOFFriction*i+1);
      m_PercussionPool.setPercussion(i,P_contact);
   }

   m_PercussionPool.clearUsedPercussionList();
   m_PercussionPool.removeUnusedPercussions();
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void  InclusionSolverNTContactOrdered<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::readFromPercussionPool(unsigned int index, const CollisionData<TLayoutConfig> * pCollData, VectorDyn & P_old)
{
   static VectorPContact P_contact;
   m_PercussionPool.getPercussion(pCollData->m_ContactTag,P_contact);
   P_old(NDOFFriction*index) = P_contact(0);
   P_old(NDOFFriction*index+1) =   P_contact(1);
   P_old(NDOFFriction*index+2) = P_contact(2);
}



#endif

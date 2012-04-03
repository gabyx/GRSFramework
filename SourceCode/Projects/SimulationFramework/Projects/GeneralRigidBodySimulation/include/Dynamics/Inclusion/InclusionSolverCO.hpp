#ifndef InclusionSolverNTContactOrdered_hpp
#define InclusionSolverNTContactOrdered_hpp

#include <fstream>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "CollisionSolver.hpp"
#include "RigidBody.hpp"
#include "PercussionPool.hpp"
#include "MatrixHelpers.hpp"
#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"
#include "InclusionSolverSettings.hpp"


#include "LogDefines.hpp"
#include "ConfigureFile.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif

#include <platformstl/performance/performance_counter.hpp>

#define USE_PERCUSSION_POOL 0


/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem.
*/
template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
class InclusionSolverCO{
public:
   DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   InclusionSolverCO(boost::shared_ptr<TCollisionSolver >  pCollisionSolver, boost::shared_ptr<TDynamicsSystem> pDynSys);

   void initializeLog( Ogre::Log* pSolverLog, boost::filesystem::path folder_path );
   void reset();
   void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
   void solveInclusionProblem( const DynamicsState<TLayoutConfig> * state_s, const DynamicsState<TLayoutConfig> * state_m, DynamicsState<TLayoutConfig> * state_e);

   PREC m_G_conditionNumber;
   PREC m_G_notDiagDominant;
   unsigned int m_iterationsNeeded;
   bool m_bConverged; unsigned int m_isFinite;
   unsigned int m_nContacts;
   bool m_bUsedGPU;
   double m_timeProx, m_proxIterationTime;

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

   boost::shared_ptr<TCollisionSolver> m_pCollisionSolver;
   boost::shared_ptr<TDynamicsSystem>  m_pDynSys;
   std::vector< boost::shared_ptr< RigidBody<TLayoutConfig> > > & m_SimBodies;
   std::vector< boost::shared_ptr< RigidBody<TLayoutConfig> > > & m_Bodies;

   TContactGraph m_ContactGraph;

   // Matrices for solving the inclusion ===========================
   PREC m_nLambdas;

   VectorDyn m_mu;

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

   MatrixDyn m_T;

   VectorDyn m_d;

   VectorDyn m_R;
   // ==========================================================================

   inline void setupRMatrix(PREC alpha);

#if HAVE_CUDA_SUPPORT == 1
   JorProxGPUVariant< JorProxGPUVariantSettingsWrapper<PREC,5,ConvexSets::RPlusAndDisk,true,300,true,10,false, TemplateHelper::Default>, ConvexSets::RPlusAndDisk > m_jorGPUVariant;
   //SorProxGPUVariant< SorProxGPUVariantSettingsWrapper<PREC,5,ConvexSets::RPlusAndDisk,true,300,true,10,true, RelaxedSorProxKernelSettings<32,ConvexSets::RPlusAndDisk> >,  ConvexSets::RPlusAndDisk > m_sorGPUVariant;
   SorProxGPUVariant< SorProxGPUVariantSettingsWrapper<PREC,1,ConvexSets::RPlusAndDisk,true,300,true,10,true,  TemplateHelper::Default >,  ConvexSets::RPlusAndDisk > m_sorGPUVariant;
#endif

   inline void doJorProx(); // These functions stay, these are the general function which are executed when we dont have GPU support! 
   inline void doSorProx(); // These functions stay, these are the general function which are executed when we dont have GPU support! 

   // Log
   Ogre::Log*	m_pSolverLog;
   std::stringstream logstream;
};



template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::InclusionSolverCO(boost::shared_ptr< TCollisionSolver >  pCollisionSolver,  boost::shared_ptr<TDynamicsSystem> pDynSys):
m_SimBodies(pCollisionSolver->m_SimBodies),
   m_Bodies(pCollisionSolver->m_Bodies)
{

   m_nSimBodies = pCollisionSolver->m_nSimBodies;
   m_nDofqObj = NDOFqObj;
   m_nDofuObj = NDOFuObj;
   m_nDofq = m_nSimBodies * m_nDofqObj;
   m_nDofu = m_nSimBodies * m_nDofuObj;
   m_nDofFriction = NDOFFriction;


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
   m_G_notDiagDominant = 0;

   m_pDynSys = pDynSys;




}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::initializeLog( Ogre::Log* pSolverLog,  boost::filesystem::path folder_path )
{
   m_pSolverLog = pSolverLog;

   
#if HAVE_CUDA_SUPPORT == 1
   
#endif
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
unsigned int InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::getNObjects()
{
   return m_nSimBodies;
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::reset()
{
   // Do a Debug check if sizes match!
   ASSERTMSG( m_SimBodies.size() * NDOFuObj == m_nDofu, "InclusionSolverCO:: Error in Dimension of System!");
   ASSERTMSG( m_SimBodies.size() * NDOFqObj == m_nDofq, "InclusionSolverCO:: Error in Dimension of System!");

   m_pDynSys->init_const_hTerm();
   m_pDynSys->init_MassMatrix();
   m_pDynSys->init_MassMatrixInv();
   
   resetForNextIter();

#if USE_PERCUSSION_POOL == 1
   reservePercussionPoolSpace(m_nSimBodies * 3);
#endif

#if HAVE_CUDA_SUPPORT == 1
   CLEARLOG;
   logstream << "Try to set GPU Device : "<< m_Settings.m_UseGPUDeviceId << endl;
   LOG(m_pSolverLog);

   CHECK_CUDA(cudaSetDevice(m_Settings.m_UseGPUDeviceId));
   cudaDeviceProp props;
   CHECK_CUDA(cudaGetDeviceProperties(&props,m_Settings.m_UseGPUDeviceId));

   CLEARLOG;
   logstream << "Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << endl;
   LOG(m_pSolverLog);
#endif


}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::resetForNextIter()
{
   m_nContacts = 0;
   m_nLambdas = 0;

   m_iterationsNeeded =0;
   m_bConverged = false;
   m_G_conditionNumber = 0;
   m_G_notDiagDominant = 0;

   m_ContactGraph.clearGraph();
}


template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::swapPercussionBuffer()
{
   std::swap(m_P_back,m_P_front);
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::resetPercussionBuffer()
{
   m_P_back = &m_P_1;
   m_P_front = &m_P_2;
}



template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::reservePercussionPoolSpace( unsigned int nExpectedContacts )
{
   m_nExpectedContacts = nExpectedContacts;
   m_PercussionPool.rehashPercussionPool(m_nExpectedContacts);
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::solveInclusionProblem(const DynamicsState<TLayoutConfig> * state_s,
   const DynamicsState<TLayoutConfig> * state_m,
   DynamicsState<TLayoutConfig> * state_e)
{

#if CoutLevelSolver>0
   CLEARLOG;
   logstream << " % -> solveInclusionProblem(): "<< endl;
   LOG(m_pSolverLog);
#endif

   // Iterate over all nodes set and assemble the matrices...
  TContactGraph::NodeList & nodes = m_ContactGraph.getNodeListRef();
  TContactGraph::Node * currentContactNode; 
   m_nContacts = (unsigned int)nodes.size();

   m_iterationsNeeded = 0;
   m_bConverged = false;
   m_isFinite = -1;
   m_bUsedGPU = false;
   m_timeProx = 0;
   m_proxIterationTime = 0;

   if(m_nContacts > 0){

      #if CoutLevelSolver>0
         CLEARLOG;
         logstream << " % nContacts = "<< m_nContacts<< endl;
         LOG(m_pSolverLog);
      #endif

      m_nLambdas = m_ContactGraph.m_nLambdas;

      // Assign Space for matrices =====================================
      m_mu.resize(m_ContactGraph.m_nFrictionParams);

      m_P_1.resize(m_nLambdas);
      m_P_2.resize(m_nLambdas);

      m_T.setZero(m_nLambdas,m_nLambdas);

      m_d.setZero(m_nLambdas);

      m_R.resize(m_nLambdas);
      // ==============================================================

      resetPercussionBuffer();
      P_back.setZero();


      // Assemble W_N and W_T and xi_N and xi_T =====================================================
      static const CollisionData<TLayoutConfig> * pCollData;

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

         //cout <<"body1 Id: "<< pCollData->m_pBody1->m_id<<", body2 Id:"<< pCollData->m_pBody2->m_id <<endl;
         /*if(m_nContacts == 4){
               cout << "4 contacts"<<endl;
         }*/

#if _DEBUG
         if(currentContactNode->m_nodeData.m_eContactModel != ContactModels::NCFContactModel){
            ASSERTMSG(false,"You use InclusionSolverCO which only supports NCFContactModel Contacts so far!");
         }
#endif

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
           

            
            // Compute G (T is used, will be later completed to T)
            // Calculate G (Start = pNode (i) , End= (*it) (j) )

            W_i_body = TContactGraph::getW_body(currentContactNode->m_nodeData,edgesBody);
            

            if(i == j){ // We are on a self referencing edge! Thats good build diagonal of G and parts of c
               W_i_bodyT_M_body = (*W_i_body).transpose() * edgesBody->m_MassMatrixInv_diag.asDiagonal();
               m_T.block<(NDOFFriction+1),(NDOFFriction+1)>((NDOFFriction+1)*i,(NDOFFriction+1)*j).noalias()  += W_i_bodyT_M_body * (*W_i_body);
               m_d.segment<NDOFFriction+1>((NDOFFriction+1)*i).noalias() +=  W_i_bodyT_M_body * edgesBody->m_h_term * m_Settings.m_deltaT + 
                                                                             currentContactNode->m_nodeData.m_I_plus_eps.asDiagonal()*( (*W_i_body).transpose() * state_s->m_SimBodyStates[bodyId].m_u );
            }                                                                
            else{
              W_j_body = TContactGraph::getW_body((*it)->m_endNode->m_nodeData, edgesBody);
              G_part = (*W_i_body).transpose() * edgesBody->m_MassMatrixInv_diag.asDiagonal() * (*W_j_body);
              m_T.block<(NDOFFriction+1),(NDOFFriction+1)>((NDOFFriction+1)*i,(NDOFFriction+1)*j).noalias() = G_part;
              m_T.block<(NDOFFriction+1),(NDOFFriction+1)>((NDOFFriction+1)*j,(NDOFFriction+1)*i).noalias() = G_part.transpose();
            }

         }
 
         // add once xi to c (d is used will be later completed to d)
         m_d.segment<NDOFFriction+1>((NDOFFriction+1)*i).noalias() +=  currentContactNode->m_nodeData.m_I_plus_eps.asDiagonal() * currentContactNode->m_nodeData.m_xi;

         
         // Fill in Percussions
#if USE_PERCUSSION_POOL == 1
         readFromPercussionPool(contactIdx,pCollData,P_back);
#endif

      }
      // =============================================================================================================


#if CALCULATE_COND_OF_G == 1
      JacobiSVD<MatrixXd> svd(m_T);
      m_G_conditionNumber = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
#endif

#if CALCULATE_DIAGDOM_OF_G == 1
      m_G_notDiagDominant = (m_T.diagonal().array().abs() < ( m_T.array().abs().matrix().rowwise().sum() - m_T.diagonal().array().abs().matrix()).array() ).count();
#endif


#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << " G= ..."<< endl << m_T.format(MyIOFormat::Matlab)<<";"<<endl;
      LOG(m_pSolverLog);
#endif

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << " c= " << m_d.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
      LOG(m_pSolverLog);
#endif

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << " P_back= "<<P_back.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
      LOG(m_pSolverLog);
#endif
      

      if( m_Settings.m_eMethod == InclusionSolverSettings<TLayoutConfig>::SOR){
         // Calculate  R_N, R_T,
         setupRMatrix(m_Settings.m_alphaSORProx);
         m_T = (-m_R).asDiagonal()*m_T;
         m_T.diagonal().array() += 1 ;
         m_d.noalias() = (-m_R).asDiagonal()*m_d;

#if MEASURE_TIME_PROX == 1
         platformstl::performance_counter counter;
         counter.start();
#endif

         doSorProx();

#if MEASURE_TIME_PROX == 1
         counter.stop();
         m_timeProx = counter.get_microseconds()*1.0e-6;
#endif
      }
      else if(m_Settings.m_eMethod == InclusionSolverSettings<TLayoutConfig>::JOR){
         // Calculate  R_N, R_T,
         setupRMatrix(m_Settings.m_alphaJORProx);
         m_T = (-m_R).asDiagonal()*m_T;
         m_T.diagonal().array() += 1 ;
         m_d.noalias() = (-m_R).asDiagonal()*m_d;

#if MEASURE_TIME_PROX == 1
         platformstl::performance_counter counter;
         counter.start();
#endif

         doJorProx();

#if MEASURE_TIME_PROX == 1
         counter.stop();
         m_timeProx = counter.get_microseconds()*1.0e-6;
#endif
      }

      if(m_Settings.m_bIsFiniteCheck){
         if(!MatrixHelpers::isfinite(P_front)){
            m_isFinite = 0;
         }
         else{
            m_isFinite = 1;
         }
         #if CoutLevelSolverWhenContact>0
                  CLEARLOG;
                  logstream << " % Solution of Prox Iteration is finite: "<< m_isFinite <<endl;
                  LOG(m_pSolverLog);
         #endif
      }

      //TODO update ContactPercussions
#if USE_PERCUSSION_POOL == 1
      updatePercussionPool(P_front);
#endif

#if CoutLevelSolverWhenContact>0
      CLEARLOG;
      logstream << " % Prox Iterations needed: "<< m_iterationsNeeded <<endl;
      LOG(m_pSolverLog);
#endif

      // Calculate u_E for each body in the state...

      static Eigen::Matrix<PREC,NDOFuObj,1> delta_u_E;
      for(unsigned int i=0; i < m_nSimBodies; i++){
         delta_u_E = m_SimBodies[i]->m_h_term * m_Settings.m_deltaT;

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
         state_e->m_SimBodyStates[i].m_u = state_s->m_SimBodyStates[i].m_u + m_SimBodies[i]->m_MassMatrixInv_diag.asDiagonal()*m_SimBodies[i]->m_h_term * m_Settings.m_deltaT;;
      }
   }

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::setupRMatrix(PREC alpha){
   PREC r_T_i;
   for(unsigned int i=0;i<m_nContacts;i++){
      m_R((NDOFFriction+1)*i) =  alpha / m_T((NDOFFriction+1)*i,(NDOFFriction+1)*i);
      r_T_i = alpha / (m_T.diagonal().segment<NDOFFriction>((NDOFFriction+1)*i+1)).maxCoeff();
      m_R((NDOFFriction+1)*i+1) = r_T_i;
      m_R((NDOFFriction+1)*i+2) = r_T_i;
   }


   #if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << " R= "<< "diag(" << m_R.transpose().format(MyIOFormat::Matlab)<<"');"<<endl;
      LOG(m_pSolverLog);
   #endif

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::doJorProx(){

   // This JOR Prox is worse then JOR prox of InclusionSolverNT which uses P_Front_N already for tangential directions.

 
#if HAVE_CUDA_SUPPORT == 1
      bool gpuSuccess = true;
      bool goOnGPU = m_nContacts >= m_jorGPUVariant.getTradeoff();

      if( m_Settings.m_bUseGPU && goOnGPU ){
#if CoutLevelSolverWhenContact>0
         m_pSolverLog->logMessage(" % Using GPU JOR...");
#endif
         m_jorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
         gpuSuccess = m_jorGPUVariant.runGPUPlain(P_front,m_T,P_back,m_d,m_mu);
         m_iterationsNeeded = m_jorGPUVariant.m_nIterGPU;
         m_proxIterationTime = m_jorGPUVariant.m_gpuIterationTime*1e-3;
         m_bUsedGPU = true;
      }

      if( !m_Settings.m_bUseGPU || !gpuSuccess || !goOnGPU){
#if CoutLevelSolverWhenContact>0
         m_pSolverLog->logMessage(" % Using CPU JOR...");
#endif
         m_jorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
         m_jorGPUVariant.runCPUEquivalentPlain(P_front,m_T,P_back,m_d,m_mu);
         m_iterationsNeeded = m_jorGPUVariant.m_nIterCPU;
         m_proxIterationTime = m_jorGPUVariant.m_cpuIterationTime*1e-3;
         m_bUsedGPU = false;
      }
      m_bConverged = (m_iterationsNeeded < m_Settings.m_MaxIter)? true : false;
#else

#if CoutLevelSolverWhenContact>0
         m_pSolverLog->logMessage(" % Using CPU JOR...");
#endif

   m_bUsedGPU = false;

      // General stupid Prox- Iteration
      while(true)
      {

         P_front.noalias() = m_T *P_back + m_d;
         Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxMulti(m_mu,P_front);

         //Calculate CancelCriteria
         m_bConverged = Numerics::cancelCriteriaValue(P_back,P_front, m_Settings.m_AbsTol, m_Settings.m_RelTol);

   #if CoutLevelSolverWhenContact>2
         CLEARLOG;
         logstream << " P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
         LOG(m_pSolverLog);
   #endif

         m_iterationsNeeded++;

         if (m_bConverged == true || m_iterationsNeeded >= m_Settings.m_MaxIter)
         {
            break;
         }
         swapPercussionBuffer();
      }

#endif

}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::doSorProx(){

   static VectorPContact PContact_back;
   static unsigned int counterConverged;
  


#if HAVE_CUDA_SUPPORT == 1
    bool gpuSuccess = true;
    bool goOnGPU = m_nContacts >= m_sorGPUVariant.getTradeoff();

   if( m_Settings.m_bUseGPU && goOnGPU){
#if CoutLevelSolverWhenContact>0
      m_pSolverLog->logMessage(" % Using GPU SOR...");
#endif
      m_sorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
      P_back.setZero();
      gpuSuccess = m_sorGPUVariant.runGPUPlain(P_front,m_T,P_back,m_d,m_mu);
      m_iterationsNeeded = m_sorGPUVariant.m_nIterGPU;
      m_proxIterationTime = m_sorGPUVariant.m_gpuIterationTime*1e-3;
      m_bUsedGPU = true;
   }
   if( !m_Settings.m_bUseGPU || !gpuSuccess || !goOnGPU){

#if CoutLevelSolverWhenContact>0
         m_pSolverLog->logMessage(" % Using CPU SOR...");
#endif
      m_sorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
      m_sorGPUVariant.runCPUEquivalentPlain(P_front,m_T,P_back,m_d,m_mu);
      m_iterationsNeeded = m_sorGPUVariant.m_nIterCPU;
      m_proxIterationTime = m_sorGPUVariant.m_cpuIterationTime*1e-3;
      m_bUsedGPU = false;
   }
   m_bConverged = (m_iterationsNeeded < m_Settings.m_MaxIter)? true : false;
#else

#if CoutLevelSolverWhenContact>0
         m_pSolverLog->logMessage(" % Using CPU SOR...");
#endif

   m_bUsedGPU = false;

   // P_back is filled with initial values, swap so P_front is filled with initial values, and only work with P_front;
   swapPercussionBuffer();

   // Prox- Iteration
   while (true)
   {
      m_bConverged = false;
      counterConverged = 0;

      // Prox all, with sequential Sor Style..
      for(unsigned int i=0; i < m_nContacts; i++){

         // Prox the contact
         PContact_back.noalias() = P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i) ; // Save last values

         // This statement looks like aliasing but it does not! All Matrices*Matrices or MAtrices*Vector get evaluated into temporary by default
         // TODO Check if there is a Aliasing effect
         P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i) = m_T.block((NDOFFriction+1)*i,0,NDOFFriction+1,m_nLambdas) * P_front + m_d.segment<NDOFFriction+1>((NDOFFriction+1)*i);

         Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(m_mu(i),P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i));
         Numerics::cancelCriteriaValue(PContact_back, P_front.segment<NDOFFriction+1>((NDOFFriction+1)*i), m_Settings.m_AbsTol, m_Settings.m_RelTol, counterConverged);

      }

#if CoutLevelSolverWhenContact>2
      CLEARLOG;
      logstream << " P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<endl;
      LOG(m_pSolverLog);
#endif

      m_iterationsNeeded++;

      if (counterConverged == m_nContacts || m_iterationsNeeded >= m_Settings.m_MaxIter)
      {
         m_bConverged = true;
         // P_front has newest values.
         break;
      }
   }
  
#endif
}

template< typename TLayoutConfig, typename TDynamicsSystem,  typename TCollisionSolver, typename TContactGraph >
void InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::updatePercussionPool(const VectorDyn & P_old )
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
void  InclusionSolverCO<TLayoutConfig, TDynamicsSystem, TCollisionSolver, TContactGraph>::readFromPercussionPool(unsigned int index, const CollisionData<TLayoutConfig> * pCollData, VectorDyn & P_old)
{
   static VectorPContact P_contact;
   m_PercussionPool.getPercussion(pCollData->m_ContactTag,P_contact);
   P_old(NDOFFriction*index) = P_contact(0);
   P_old(NDOFFriction*index+1) =   P_contact(1);
   P_old(NDOFFriction*index+2) = P_contact(2);
}



#endif

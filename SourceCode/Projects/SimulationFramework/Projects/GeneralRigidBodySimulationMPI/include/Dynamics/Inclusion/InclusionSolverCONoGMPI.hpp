#ifndef InclusionSolverNTContactOrderedNoGMPI_hpp
#define InclusionSolverNTContactOrderedNoGMPI_hpp


#include <iostream>
#include <fstream>


#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "CollisionSolverMPI.hpp"
#include "RigidBody.hpp"
#include "PercussionPool.hpp"
#include "MatrixHelpers.hpp"
#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"
#include "InclusionSolverSettings.hpp"
#include "ContactGraph.hpp"

#include "LogDefines.hpp"
#include "ConfigureFile.hpp"

#include "SimpleLogger.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif

#include <boost/timer/timer.hpp>



/**
* @ingroup Inclusion
* @brief The inclusion solver for an ordered problem. Does not assemble G. Iterates over Contact Graph in a SOR fashion!
*/
template< typename TInclusionSolverConfig >
class InclusionSolverCONoG {
public:

    DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES_OF(TInclusionSolverConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InclusionSolverCONoG(boost::shared_ptr<CollisionSolverType >  pCollisionSolver, boost::shared_ptr<DynamicsSystemType> pDynSys);

    void initializeLog( Logging::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
    void solveInclusionProblem();


    std::string getIterationStats();
    unsigned int m_iterationsNeeded;
    bool m_bConverged;
    unsigned int m_isFinite;
    unsigned int m_nContacts;
    bool m_bUsedGPU;
    double m_timeProx, m_proxIterationTime;

    ContactParameterMap<RigidBodyType> m_ContactParameterMap;

    PercussionPool<LayoutConfigType> m_PercussionPool;

//    void reservePercussionPoolSpace(unsigned int nExpectedContacts);
//    void readFromPercussionPool(unsigned int index, const CollisionData<LayoutConfigType> * pCollData, VectorDyn & P_old);
//    void updatePercussionPool(const VectorDyn & P_old ) ;


    InclusionSolverSettings<LayoutConfigType> m_Settings;

    unsigned int getNObjects();

protected:
    unsigned int m_nDofq, m_nDofu, m_nDofqObj, m_nDofuObj, m_nDofFriction, m_nSimBodies;

    unsigned int m_nExpectedContacts;

    boost::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    boost::shared_ptr<DynamicsSystemType>  m_pDynSys;


    typename DynamicsSystemType::RigidBodySimPtrListType & m_SimBodies;

    typename DynamicsSystemType::RigidBodyNotAniPtrListType & m_Bodies;

    typedef ContactGraph<RigidBodyType,ContactGraphMode::ForIteration> ContactGraphType;
    ContactGraphType m_ContactGraph;

    void integrateAllBodyVelocities();
    void initContactGraphForIteration(PREC alpha);

    inline void doJorProx();

    inline void doSorProx();
    inline void sorProxOverAllNodes();
    //inline void sorProxOverAllBodies(bool checkConvergence);
    inline void sorProxOverAllNodesLast();
    // Log
    Logging::Log*	m_pSolverLog;
    std::stringstream logstream;
};




template< typename TInclusionSolverConfig >
InclusionSolverCONoG<TInclusionSolverConfig>::InclusionSolverCONoG(boost::shared_ptr< CollisionSolverType >  pCollisionSolver,  boost::shared_ptr<DynamicsSystemType > pDynSys):
    m_SimBodies(pCollisionSolver->m_SimBodies),
    m_Bodies(pCollisionSolver->m_Bodies) {

    m_nSimBodies = pCollisionSolver->m_nSimBodies;
    m_nDofqObj = NDOFqObj;
    m_nDofuObj = NDOFuObj;
    m_nDofq = m_nSimBodies * m_nDofqObj;
    m_nDofu = m_nSimBodies * m_nDofuObj;


    m_pCollisionSolver = pCollisionSolver;

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->m_ContactDelegateList.addContactDelegate(
        ContactDelegateList<RigidBodyType>::ContactDelegate::template from_method< ContactGraphType,  &ContactGraphType::addNode>(&m_ContactGraph)
    );

    m_nContacts = 0;

    m_iterationsNeeded =0;
    m_bConverged = true;
    m_pDynSys = pDynSys;




}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::initializeLog( Logging::Log
        * pSolverLog,  boost::filesystem::path folder_path ) {
    m_pSolverLog = pSolverLog;


#if HAVE_CUDA_SUPPORT == 1

#endif
}


template< typename TInclusionSolverConfig >
unsigned int InclusionSolverCONoG<TInclusionSolverConfig>::getNObjects() {
    return m_nSimBodies;
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::reset() {
    // Do a Debug check if sizes match!
    ASSERTMSG( m_SimBodies.size() * NDOFuObj == m_nDofu, "InclusionSolverCONoG:: Error in Dimension of System!");
    ASSERTMSG( m_SimBodies.size() * NDOFqObj == m_nDofq, "InclusionSolverCONoG:: Error in Dimension of System!");

    m_pDynSys->init_const_hTerm();
    m_pDynSys->init_MassMatrix();
    m_pDynSys->init_MassMatrixInv();

    resetForNextIter();

#if HAVE_CUDA_SUPPORT == 1
    LOG(m_pSolverLog, << "Try to set GPU Device : "<< m_Settings.m_UseGPUDeviceId << std::endl;);

    CHECK_CUDA(cudaSetDevice(m_Settings.m_UseGPUDeviceId));
    cudaDeviceProp props;
    CHECK_CUDA(cudaGetDeviceProperties(&props,m_Settings.m_UseGPUDeviceId));

    LOG(m_pSolverLog, <<  "Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << std::endl;);
#endif


}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::resetForNextIter() {

    m_nContacts = 0;
    m_iterationsNeeded =0;

    m_bConverged = true;


    m_ContactGraph.clearGraph();
}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::solveInclusionProblem() {

#if CoutLevelSolver>0
    LOG(m_pSolverLog, <<  " % -> solveInclusionProblem(): "<< std::endl;);
#endif

    // Iterate over all nodes set and assemble the matrices...
    typename ContactGraphType::NodeListType & nodes = m_ContactGraph.getNodeListRef();
    m_nContacts = (unsigned int)nodes.size();

    // Standart values
    m_iterationsNeeded = 0;
    m_bConverged = false; // Set true later, if one node is not converged then 0! and we do one more loop
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;


    // Integrate all bodies to u_e
    // u_E = u_S + M^⁻1 * h * deltaT
    if(m_nContacts == 0){
        integrateAllBodyVelocities();
    }
    else{

    // Solve Inclusion

        // Fill in Percussions
#if USE_PERCUSSION_POOL == 1
        //readFromPercussionPool(contactIdx,pCollData,P_back);
#endif

#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog, <<  " % nContacts: "<< m_nContacts <<std::endl;);
#endif



        // =============================================================================================================
        if( m_Settings.m_eMethod == InclusionSolverSettings<LayoutConfigType>::SOR) {

#if MEASURE_TIME_PROX == 1
            boost::timer::cpu_timer counter;
            counter.start();
#endif

            initContactGraphForIteration(m_Settings.m_alphaSORProx);
            doSorProx();

#if MEASURE_TIME_PROX == 1
            counter.stop();
            m_timeProx = ((double)counter.elapsed().wall) * 1e-9;
#endif

        } else if(m_Settings.m_eMethod == InclusionSolverSettings<LayoutConfigType>::JOR) {

#if MEASURE_TIME_PROX == 1
            boost::timer::cpu_timer counter;
            counter.start();
#endif

            initContactGraphForIteration(m_Settings.m_alphaJORProx);
            ASSERTMSG(false,"Jor Algorithm has not been implemented yet");
//            doJorProx();

#if MEASURE_TIME_PROX == 1
            counter.stop();
            m_timeProx = ((double)counter.elapsed().wall) * 1e-9;
#endif
        }

        if(m_Settings.m_bIsFiniteCheck) {

            // TODO CHECK IF finite!

#if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog, <<  " % Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
#endif
        }

        //TODO update ContactPercussions
#if USE_PERCUSSION_POOL == 1
        //updatePercussionPool(P_front);
#endif

#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog, <<  " % Prox Iterations needed: "<< m_iterationsNeeded <<std::endl;);
#endif
    }

}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::doJorProx() {
    ASSERTMSG(false,"InclusionSolverCONoG:: JOR Prox iteration not implemented!");
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::integrateAllBodyVelocities() {

    typename DynamicsSystemType::RigidBodySimPtrListType::iterator bodyIt;

    for( bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        (*bodyIt)->m_pSolverData->m_uBuffer.m_Front += (*bodyIt)->m_pSolverData->m_uBuffer.m_Back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
    }
}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::initContactGraphForIteration(PREC alpha) {


    // Calculates b vector for all nodes, u_0, R_ii, ...
    typename ContactGraphType::NodeListType &nodes = m_ContactGraph.getNodeListRef();
    for( typename ContactGraphType::NodeListIteratorType contactIt = nodes.begin(); contactIt != nodes.end(); contactIt++) {

        // Assert ContactModel
        ASSERTMSG((*contactIt)->m_nodeData.m_eContactModel == ContactModels::NCFContactModel
                  ,"You use InclusionSolverCONoG which only supports NCFContactModel Contacts so far!");

        typename ContactGraphType::NodeDataType & nodeData = (*contactIt)->m_nodeData;

        // Get lambda from percussion pool otherwise set to zero
        // TODO
        nodeData.m_LambdaBack.setZero();

        // (1+e)*xi -> b
        nodeData.m_b = nodeData.m_I_plus_eps.asDiagonal() * nodeData.m_xi;

        // u_0 , calculate const b
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED) {

            // m_Back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u1BufferPtr->m_Front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_Back /* m_u_s */ ;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {

            // m_Back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u2BufferPtr->m_Front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u1BufferPtr->m_Back;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }



        // Calculate R_ii
        nodeData.m_R_i_inv_diag(0) = alpha / nodeData.m_G_ii(0,0);
        PREC r_T = alpha / (nodeData.m_G_ii.diagonal().template tail<2>()).maxCoeff();
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;

#if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog,   << " nodeData.m_b"<< nodeData.m_b <<std::endl
            << " nodeData.m_G_ii"<< nodeData.m_G_ii <<std::endl
            << " nodeData.m_R_i_inv_diag"<< nodeData.m_R_i_inv_diag <<std::endl;);
#endif

#if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, <<  " nodeData.m_mu"<< nodeData.m_mu <<std::endl;);
#endif

    }

    // Integrate all bodies!
    typename DynamicsSystemType::RigidBodySimPtrListType::iterator bodyIt;

    for( bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        (*bodyIt)->m_pSolverData->m_uBuffer.m_Front += (*bodyIt)->m_pSolverData->m_uBuffer.m_Back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
        (*bodyIt)->m_pSolverData->m_uBuffer.m_Back = (*bodyIt)->m_pSolverData->m_uBuffer.m_Front; // Used for cancel criteria
    }
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::doSorProx() {

#if CoutLevelSolverWhenContact>2
    LOG(m_pSolverLog, << " u_e = [ ");

    for(int i=0; i< m_SimBodies.size(); i++) {
        LOG(m_pSolverLog, << "Back: \t" << m_SimBodies[i]->m_pSolverData->m_uBuffer.m_Back.transpose() <<std::endl);
        LOG(m_pSolverLog, << "Front: \t" <<m_SimBodies[i]->m_pSolverData->m_uBuffer.m_Front.transpose()<<std::endl);
    }
    LOG(m_pSolverLog, << " ]" << std::endl);
#endif

    // General stupid Prox- Iteration
    while(true) {

        m_bConverged = true;

        sorProxOverAllNodes(); // Do one Sor Prox Iteration

#if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, << std::endl<< "Next iteration: "<< m_iterationsNeeded <<"=========================" << std::endl<< std::endl<<" u_e: \t");
        for(int i=0; i< m_SimBodies.size(); i++) {
            LOG(m_pSolverLog, << m_SimBodies[i]->m_pSolverData->m_uBuffer.m_Front.transpose());
        }
        LOG(m_pSolverLog, <<""<< std::endl);
#endif

        m_iterationsNeeded++;

        if ( (m_bConverged == true || m_iterationsNeeded >= m_Settings.m_MaxIter) && m_iterationsNeeded >= m_Settings.m_MinIter) {

#if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog, << " converged = "<<m_bConverged<< "\t"<< "iterations: " <<m_iterationsNeeded <<" / "<<  m_Settings.m_MaxIter<< std::endl;);
#endif
            break;
        }
    }


}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::sorProxOverAllNodes() {

    typename ContactGraphType::NodeListIteratorType nodeIt;

    int nodeCounter = 0;

    bool converged = true;

    VectorDyn uCache1,uCache2;
    typename ContactGraphType::NodeListType &nodes = m_ContactGraph.getNodeListRef();
    for(nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++) {

        nodeCounter++;
        converged = true;

        typename ContactGraphType::NodeDataType & nodeData = (*nodeIt)->m_nodeData;

        if( nodeData.m_eContactModel == ContactModels::NCFContactModel ) {

#if CoutLevelSolverWhenContact>2
            *m_pSolverLog, << "Node: " << nodeCounter <<"====================="<<  std::endl;
#endif
            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_Front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_Front;
            }

#if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, <<"chi: " << nodeData.m_LambdaFront.transpose() << std::endl);
#endif

            nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
            nodeData.m_LambdaFront += nodeData.m_LambdaBack;
            //Prox

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(
                nodeData.m_mu(0),
                nodeData.m_LambdaFront.template head<ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension>()
            );

#if CoutLevelSolverWhenContact>2
            *m_pSolverLog <<"Lambda Back: "  << nodeData.m_LambdaBack.transpose() << std::endl;
            *m_pSolverLog <<"Lambda Front: " << nodeData.m_LambdaFront.transpose() << std::endl;
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol)){
              *m_pSolverLog <<"Lambda converged"<<std::endl;
            }
#endif

            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_Front;
                nodeData.m_u1BufferPtr->m_Front = nodeData.m_u1BufferPtr->m_Front + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

           #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, <<"Node: " << nodeData.m_u1BufferPtr->m_Front.transpose() << std::endl);
           #endif


                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InVelocityLocal) {
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {
                        converged = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_Front,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                        if(!converged) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InEnergyLocalMix){
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {
                        converged = Numerics::cancelCriteriaMatrixNorm(   uCache1,
                                                                          nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                                          nodeData.m_LambdaBack,
                                                                          nodeData.m_LambdaFront,
                                                                          nodeData.m_G_ii,
                                                                          m_Settings.m_AbsTol,
                                                                          m_Settings.m_RelTol);
                        if(!converged) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }

            }
            // SECOND BODY
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_Front;
                nodeData.m_u2BufferPtr->m_Front = nodeData.m_u2BufferPtr->m_Front  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InVelocityLocal) {
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {
                        converged = Numerics::cancelCriteriaValue(uCache2,
                                                                  nodeData.m_u2BufferPtr->m_Front,
                                                                  m_Settings.m_AbsTol,
                                                                  m_Settings.m_RelTol);
                        if(!converged) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InEnergyLocalMix){
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {
                        converged = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                                          nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                                          nodeData.m_LambdaBack,
                                                                          nodeData.m_LambdaFront,
                                                                          nodeData.m_G_ii,
                                                                          m_Settings.m_AbsTol,
                                                                          m_Settings.m_RelTol);
                        if(!converged) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }
            }

            if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InLambda) {
                if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {
                    converged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                    if(!converged) {
                        //converged stays false;
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }

            // Swap Lambdas, but dont swap Velocities...
            // Swap velocities when we finished ONE Sor Prox Iteration! (very important!)
            nodeData.swapLambdas();


        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }
    } // Move over all nodes, end of Sor Prox



    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
    if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InVelocity) {
        typename ContactGraphType::BodyToContactsListIteratorType it;
        //std::cout << "Bodies: " << m_ContactGraph.m_SimBodyToContactsList.size() << std::endl;
        for(it=m_ContactGraph.m_SimBodyToContactsList.begin(); it !=m_ContactGraph.m_SimBodyToContactsList.end(); it++) {
            if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {
                //std::cout << "before Criteria"<<std::endl;
                //std::cout <<"new "<< it->first->m_pSolverData->m_uBuffer.m_Front.transpose() << std::endl;
                //std::cout <<"old "<< it->first->m_pSolverData->m_uBuffer.m_Back.transpose() << std::endl;
                converged = Numerics::cancelCriteriaValue(it->first->m_pSolverData->m_uBuffer.m_Back, // these are the old values (got switched)
                            it->first->m_pSolverData->m_uBuffer.m_Front, // these are the new values (got switched)
                            m_Settings.m_AbsTol,
                            m_Settings.m_RelTol);
                //std::cout << "after Criteria"<<std::endl;
                if(!converged) {
                    m_bConverged=false;
                }

            } else {
                m_bConverged=false;
            }
            // Do not switch Velocities (for next Sor Prox Iteration)
            // Just fill back buffer with new values!
            it->first->m_pSolverData->m_uBuffer.m_Back = it->first->m_pSolverData->m_uBuffer.m_Front;
        }
    }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InEnergyVelocity){
        typename ContactGraphType::BodyToContactsListIteratorType it;
        for(it=m_ContactGraph.m_SimBodyToContactsList.begin(); it !=m_ContactGraph.m_SimBodyToContactsList.end(); it++) {
            if(m_iterationsNeeded >= m_Settings.m_MinIter && converged) {

                converged = Numerics::cancelCriteriaMatrixNorm( it->first->m_pSolverData->m_uBuffer.m_Back, // these are the old values (got switched)
                                                                it->first->m_pSolverData->m_uBuffer.m_Front, // these are the new values (got switched)
                                                                it->first->m_MassMatrix_diag,
                                                                m_Settings.m_AbsTol,
                                                                m_Settings.m_RelTol);
                if(!converged) {
                    m_bConverged=false;
                }

            } else {
                m_bConverged=false;
            }
            // Do not switch Velocities (for next Sor Prox Iteration)
            // Just fill back buffer with new values! for next global iteration
            it->first->m_pSolverData->m_uBuffer.m_Back = it->first->m_pSolverData->m_uBuffer.m_Front;
        }
    }


}

//template< typename TInclusionSolverConfig >
//void InclusionSolverCONoG<TInclusionSolverConfig>::sorProxOverAllNodesLast(DynamicsState<LayoutConfigType> * state_e) {
//    // Apply converged u_e to global u_e simulated bodies!
//    // Reset all flags!
//    typename ContactGraphType::BodyToContactsListIteratorType it;
//    for(it=m_ContactGraph.m_SimBodyToContactsList.begin(); it !=m_ContactGraph.m_SimBodyToContactsList.end(); it++) {
//        state_e->m_SimBodyStates[it->first->m_id].m_u = it->first->m_pSolverData->m_uBuffer.m_Front;
//        it->first->m_pSolverData->reset();
//    }
//}



template< typename TInclusionSolverConfig >
std::string  InclusionSolverCONoG<TInclusionSolverConfig>::getIterationStats() {
    std::stringstream s;

    s   << m_bUsedGPU<<"\t"
    << m_nContacts<<"\t"
    << m_iterationsNeeded<<"\t"
    << m_bConverged<<"\t"
    << m_isFinite<<"\t"
    << m_timeProx<<"\t"
    << m_proxIterationTime<<"\t"
    << m_pDynSys->m_CurrentStateEnergy <<"\t"
    << -1<<"\t" //No m_G_conditionNumber
    << -1<<"\t" //No m_G_notDiagDominant
    << m_PercussionPool.getPoolSize()<<std::endl;
    return s.str();
}
#endif

#ifndef InclusionSolverNTContactOrderedNoG_hpp
#define InclusionSolverNTContactOrderedNoG_hpp


#include <iostream>
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
#include "ContactGraph.hpp"

#include "LogDefines.hpp"
#include "ConfigureFile.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif

#include <platformstl/performance/performance_counter.hpp>



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

    void initializeLog( Ogre::Log* pSolverLog, boost::filesystem::path folder_path );
    void reset();
    void resetForNextIter(); // Is called each iteration in the timestepper, so that the InclusionSolver is able to reset matrices which are dynamically added to during the iteration! (like, h term)
    void solveInclusionProblem( const DynamicsState<LayoutConfigType> * state_s, const DynamicsState<LayoutConfigType> * state_m, DynamicsState<LayoutConfigType> * state_e);


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

    typedef std::vector< boost::shared_ptr< RigidBodyType > > RigidBodySimPtrListType;
    RigidBodySimPtrListType & m_SimBodies;
    typedef std::vector< boost::shared_ptr< RigidBodyType > > RigidBodyNotAniPtrListType;
    RigidBodyNotAniPtrListType & m_Bodies;

    typedef ContactGraph<RigidBodyType,ContactGraphMode::ForIteration> ContactGraphType;
    ContactGraphType m_ContactGraph;

    void integrateAllBodyVelocities(const DynamicsState<LayoutConfigType> * state_s , DynamicsState<LayoutConfigType> * state_e);
    void initContactGraphForIteration(const DynamicsState<LayoutConfigType> * state_s , DynamicsState<LayoutConfigType> * state_e, PREC alpha);

    inline void doJorProx();

    inline void doSorProx(DynamicsState<LayoutConfigType> * state_e);
    inline void sorProxOverAllNodes(DynamicsState<LayoutConfigType> * state_e);
    // Log
    Ogre::Log*	m_pSolverLog;
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
void InclusionSolverCONoG<TInclusionSolverConfig>::initializeLog( Ogre::Log* pSolverLog,  boost::filesystem::path folder_path ) {
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
    CLEARLOG;
    logstream << "Try to set GPU Device : "<< m_Settings.m_UseGPUDeviceId << std::endl;
    LOG(m_pSolverLog);

    CHECK_CUDA(cudaSetDevice(m_Settings.m_UseGPUDeviceId));
    cudaDeviceProp props;
    CHECK_CUDA(cudaGetDeviceProperties(&props,m_Settings.m_UseGPUDeviceId));

    CLEARLOG;
    logstream << "Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << std::endl;
    LOG(m_pSolverLog);
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
void InclusionSolverCONoG<TInclusionSolverConfig>::solveInclusionProblem(const DynamicsState<LayoutConfigType> * state_s,
        const DynamicsState<LayoutConfigType> * state_m,
        DynamicsState<LayoutConfigType> * state_e) {

#if CoutLevelSolver>0
    CLEARLOG;
    logstream << " % -> solveInclusionProblem(): "<< std::endl;
    LOG(m_pSolverLog);
#endif

    // Iterate over all nodes set and assemble the matrices...
    typename ContactGraphType::NodeListType & nodes = m_ContactGraph.getNodeListRef();
    typename ContactGraphType::NodeType * currentContactNode;
    m_nContacts = (unsigned int)nodes.size();

    m_iterationsNeeded = 0;
    m_bConverged = true; // Set true, if one node is not converged then 0! and we do one more loop
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;


    // Integrate all bodies to u_e
    // u_E = u_S + M^⁻1 * h * deltaT
    integrateAllBodyVelocities(state_s,state_e);


    // Solve Inclusion
    if(m_nContacts > 0) {
        // Fill in Percussions
#if USE_PERCUSSION_POOL == 1
        //readFromPercussionPool(contactIdx,pCollData,P_back);
#endif





        // =============================================================================================================
        if( m_Settings.m_eMethod == InclusionSolverSettings<LayoutConfigType>::SOR) {

#if MEASURE_TIME_PROX == 1
            platformstl::performance_counter counter;
            counter.start();
#endif

            initContactGraphForIteration(state_s,state_e, m_Settings.m_alphaSORProx);
            doSorProx(state_e);

#if MEASURE_TIME_PROX == 1
            counter.stop();
            m_timeProx = counter.get_microseconds()*1.0e-6;
#endif

        } else if(m_Settings.m_eMethod == InclusionSolverSettings<LayoutConfigType>::JOR) {

#if MEASURE_TIME_PROX == 1
            platformstl::performance_counter counter;
            counter.start();
#endif

            initContactGraphForIteration(state_s,state_e, m_Settings.m_alphaJORProx);
            doJorProx();

#if MEASURE_TIME_PROX == 1
            counter.stop();
            m_timeProx = counter.get_microseconds()*1.0e-6;
#endif
        }

        if(m_Settings.m_bIsFiniteCheck) {

            // TODO CHECK IF finite!

#if CoutLevelSolverWhenContact>0
            CLEARLOG;
            logstream << " % Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;
            LOG(m_pSolverLog);
#endif
        }

        //TODO update ContactPercussions
#if USE_PERCUSSION_POOL == 1
        //updatePercussionPool(P_front);
#endif

#if CoutLevelSolverWhenContact>0
        CLEARLOG;
        logstream << " % Prox Iterations needed: "<< m_iterationsNeeded <<std::endl;
        LOG(m_pSolverLog);
#endif
    }

}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::doJorProx() {
    ASSERTMSG(false,"InclusionSolverCONoG:: JOR Prox iteration not implemented!");
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::integrateAllBodyVelocities(const DynamicsState<LayoutConfigType> * state_s , DynamicsState<LayoutConfigType> * state_e){

        typename DynamicsState<LayoutConfigType>::RigidBodyStateListType::iterator stateItE;
        typename DynamicsState<LayoutConfigType>::RigidBodyStateListType::const_iterator stateItS;
        typename RigidBodySimPtrListType::iterator bodyIt;

        bodyIt = m_SimBodies.begin();
        stateItS = state_s->m_SimBodyStates.begin();
        stateItE = state_e->m_SimBodyStates.begin();

        for(; stateItE != state_e->m_SimBodyStates.end(); ) {
            stateItE->m_u = stateItS->m_u + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;

            // TODO Write to Buffer
            (*bodyIt)->m_pSolverData->m_uBuffer.m_Back = stateItE->m_u;

            // Move iterators
            bodyIt++;
            stateItS++;
            stateItE++;
        }

}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::initContactGraphForIteration(const DynamicsState<LayoutConfigType> * state_s, DynamicsState<LayoutConfigType> * state_e,  PREC alpha) {

    // Calculates b vector for all nodes, u_0, R_ii, ...
    for( typename ContactGraphType::NodeListIteratorType contactIt = m_ContactGraph.m_nodes.begin(); contactIt != m_ContactGraph.m_nodes.end(); contactIt++) {

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
        int bodyNr ;
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED) {
            bodyNr = nodeData.m_pCollData->m_pBody1->m_id;

            // m_Back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u1BufferPtr->m_Back +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * state_s->m_SimBodyStates[bodyNr].m_u;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
            bodyNr = nodeData.m_pCollData->m_pBody2->m_id;

            // m_Back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u2BufferPtr->m_Back +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() * state_s->m_SimBodyStates[bodyNr].m_u;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }



        // Calculate R_ii
        nodeData.m_R_i_inv_diag(0) = alpha / nodeData.m_G_ii(0,0);
        PREC r_T = alpha / (nodeData.m_G_ii.diagonal().template tail<2>()).maxCoeff();
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;

#if CoutLevelSolverWhenContact>2
        CLEARLOG;
        logstream << " nodeData.m_b"<< nodeData.m_b <<std::endl;
        logstream << " nodeData.m_G_ii"<< nodeData.m_G_ii <<std::endl;
        logstream << " nodeData.m_R_i_inv_diag"<< nodeData.m_R_i_inv_diag <<std::endl;
        LOG(m_pSolverLog);
#endif

#if CoutLevelSolverWhenContact>2
        CLEARLOG;
        logstream << " nodeData.m_b"<< nodeData.m_b <<std::endl;
        LOG(m_pSolverLog);
#endif



    }
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::doSorProx(DynamicsState<LayoutConfigType> * state_e) {

    // General stupid Prox- Iteration
    while(true) {

        m_bConverged = true;

        sorProxOverAllNodes(state_e);

        m_iterationsNeeded++;

        if (m_bConverged == true || m_iterationsNeeded >= m_Settings.m_MaxIter) {

#if CoutLevelSolverWhenContact>2
            CLEARLOG;
            logstream << " converged = "<<m_bConverged<< "\t"<< "iterations:" <<m_iterationsNeeded <<"/"<<  m_Settings.m_MaxIter<< std::endl;
            LOG(m_pSolverLog);
#endif
            break;
        }
    }


}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::sorProxOverAllNodes(DynamicsState<LayoutConfigType> * state_e) {

    typename ContactGraphType::NodeListIteratorType nodeIt;


    for(nodeIt = m_ContactGraph.m_nodes.begin(); nodeIt != m_ContactGraph.m_nodes.end(); nodeIt++) {

        typename ContactGraphType::NodeDataType & nodeData = (*nodeIt)->m_nodeData;

        if( nodeData.m_eContactModel == ContactModels::NCFContactModel ) {

            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_Back ;
            }

            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_Back;
            }

            nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
            nodeData.m_LambdaFront += nodeData.m_LambdaBack;

            //Prox

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(
                nodeData.m_mu(0),
                nodeData.m_LambdaFront.template head<ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension>()
            );


            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            bool converged = false;
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_u1BufferPtr->m_Front = nodeData.m_u1BufferPtr->m_Back + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                // Write directly to global
                state_e->m_SimBodyStates[nodeData.m_pCollData->m_pBody1->m_id].m_u = nodeData.m_u1BufferPtr->m_Front;

                //Check first
                converged = Numerics::cancelCriteriaValue(nodeData.m_u1BufferPtr->m_Back, nodeData.m_u1BufferPtr->m_Front, m_Settings.m_AbsTol, m_Settings.m_RelTol);
            }

            // SECOND BONDY
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_u2BufferPtr->m_Front = nodeData.m_u2BufferPtr->m_Back  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                state_e->m_SimBodyStates[nodeData.m_pCollData->m_pBody2->m_id].m_u = nodeData.m_u2BufferPtr->m_Front;

                // If first not converged dont check second
                if(converged) {
                    //Check second
                    converged = Numerics::cancelCriteriaValue(nodeData.m_u2BufferPtr->m_Back, nodeData.m_u2BufferPtr->m_Front, m_Settings.m_AbsTol, m_Settings.m_RelTol);
                }
            }

            // Check abortion criteria in u


            if(!converged) {
                // If only one nodeData is not converged set the flag to false!
                m_bConverged = false;
            }

            // Swap stuff
            nodeData.swapLambdas();
            nodeData.swapVelocities();


        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }

    }
}


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

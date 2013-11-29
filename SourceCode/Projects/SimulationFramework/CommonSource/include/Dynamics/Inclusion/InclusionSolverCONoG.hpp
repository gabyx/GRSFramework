#ifndef InclusionSolverNTContactOrderedNoG_hpp
#define InclusionSolverNTContactOrderedNoG_hpp


#include <iostream>
#include <fstream>


#include <boost/shared_ptr.hpp>

#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "CollisionSolver.hpp"
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
    ~InclusionSolverCONoG();

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



    PercussionPool<LayoutConfigType> m_PercussionPool;

//    void reservePercussionPoolSpace(unsigned int nExpectedContacts);
//    void readFromPercussionPool(unsigned int index, const CollisionData<LayoutConfigType> * pCollData, VectorDyn & P_old);
//    void updatePercussionPool(const VectorDyn & P_old ) ;

    InclusionSolverSettings<LayoutConfigType> m_Settings;


protected:


    unsigned int m_nExpectedContacts;

    boost::shared_ptr<CollisionSolverType> m_pCollisionSolver;
    boost::shared_ptr<DynamicsSystemType>  m_pDynSys;

    typename DynamicsSystemType::RigidBodySimContainerType & m_SimBodies;
    typename DynamicsSystemType::RigidBodyNotAniContainer & m_Bodies;

    typedef ContactGraph<RigidBodyType,ContactGraphMode::ForIteration> ContactGraphType;
    ContactGraphType m_ContactGraph;

    void integrateAllBodyVelocities();
    void initContactGraphForIteration(PREC alpha);

    inline void doJorProx();

    inline void doSorProx();
    inline void sorProxOverAllNodes();
    SorProxStepNodeVisitor<RigidBodyType> * m_pSorProxStepNodeVisitor;
    SorProxInitNodeVisitor<RigidBodyType> * m_pSorProxInitNodeVisitor;

    // Log
    Logging::Log *m_pSolverLog, *m_pSimulationLog;
    std::stringstream logstream;
};




template< typename TInclusionSolverConfig >
InclusionSolverCONoG<TInclusionSolverConfig>::InclusionSolverCONoG(boost::shared_ptr< CollisionSolverType >  pCollisionSolver,  boost::shared_ptr<DynamicsSystemType > pDynSys):
    m_SimBodies(pCollisionSolver->m_SimBodies),
    m_Bodies(pCollisionSolver->m_Bodies),
    m_ContactGraph(&(pDynSys->m_ContactParameterMap)){

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }

    m_pCollisionSolver = pCollisionSolver;

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->m_ContactDelegateList.addContactDelegate(
        ContactDelegateList<RigidBodyType>::ContactDelegate::template from_method< ContactGraphType,  &ContactGraphType::addNode>(&m_ContactGraph)
    );

    m_nContacts = 0;

    m_iterationsNeeded =0;
    m_bConverged = true;
    m_pDynSys = pDynSys;

    //Make a new Sor Prox Visitor (takes references from these class member)
    m_pSorProxStepNodeVisitor = new SorProxStepNodeVisitor<RigidBodyType>(m_Settings,m_bConverged,m_iterationsNeeded);
    m_pSorProxInitNodeVisitor = new SorProxInitNodeVisitor<RigidBodyType>();
}

template< typename TInclusionSolverConfig >
InclusionSolverCONoG<TInclusionSolverConfig>::~InclusionSolverCONoG(){
    delete m_pSorProxStepNodeVisitor;
    delete m_pSorProxInitNodeVisitor;
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::initializeLog( Logging::Log * pSolverLog,  boost::filesystem::path folder_path ) {

    m_pSolverLog = pSolverLog;
    m_pSorProxStepNodeVisitor->setLog(m_pSolverLog);
    m_pSorProxInitNodeVisitor->setLog(m_pSolverLog);

    #if HAVE_CUDA_SUPPORT == 1

    #endif
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::reset() {

    resetForNextIter();

#if HAVE_CUDA_SUPPORT == 1
    LOG(m_pSimulationLog, "Try to set GPU Device : "<< m_Settings.m_UseGPUDeviceId << std::endl;);

    CHECK_CUDA(cudaSetDevice(m_Settings.m_UseGPUDeviceId));
    cudaDeviceProp props;
    CHECK_CUDA(cudaGetDeviceProperties(&props,m_Settings.m_UseGPUDeviceId));

    LOG(m_pSimulationLog,  "Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << std::endl;);
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

#if CoutLevelSolver>1
    LOG(m_pSolverLog,  " % -> solveInclusionProblem(): "<< std::endl;);
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
            LOG(m_pSolverLog,  " % nContacts: "<< m_nContacts <<std::endl;);
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
                LOG(m_pSolverLog,  " % Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
            #endif
        }

        //TODO update ContactPercussions
        #if USE_PERCUSSION_POOL == 1
            //updatePercussionPool(P_front);
        #endif

#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog,  " % Prox Iterations needed: "<< m_iterationsNeeded <<std::endl;);
#endif
    }

}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::doJorProx() {
    ASSERTMSG(false,"InclusionSolverCONoG:: JOR Prox iteration not implemented!");
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::integrateAllBodyVelocities() {
    for( auto bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
    }
}


template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::initContactGraphForIteration(PREC alpha) {

    // Calculates b vector for all nodes, u_0, R_ii, ...
    m_pSorProxInitNodeVisitor->setParams(alpha);
    m_ContactGraph.applyNodeVisitor(*m_pSorProxInitNodeVisitor);

    // Integrate all bodies!
    for( auto bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
        (*bodyIt)->m_pSolverData->m_uBuffer.m_back = (*bodyIt)->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
    }
}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::doSorProx() {

    #if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, " u_e = [ ");
        for(auto it = m_SimBodies.begin(); it != m_SimBodies.end(); it++) {
            LOG(m_pSolverLog, "Back: \t" << (*it)->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl);
            LOG(m_pSolverLog, "Front: \t" <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
        }
        LOG(m_pSolverLog, " ]" << std::endl);
    #endif

    // General stupid Prox- Iteration
    while(true) {

        m_bConverged = true;

        sorProxOverAllNodes(); // Do one Sor Prox Iteration

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, std::endl<< "Next iteration: "<< m_iterationsNeeded <<"=========================" << std::endl<< std::endl<<" u_e: \t");
            for(auto it = m_SimBodies.begin(); it != m_SimBodies.end(); it++) {
                LOG(m_pSolverLog, (*it)->m_pSolverData->m_uBuffer.m_front.transpose());
            }
            LOG(m_pSolverLog,""<< std::endl);
        #endif

        m_iterationsNeeded++;

        if ( (m_bConverged == true || m_iterationsNeeded >= m_Settings.m_MaxIter) && m_iterationsNeeded >= m_Settings.m_MinIter) {
            #if CoutLevelSolverWhenContact>0
                LOG(m_pSolverLog, " converged = "<<m_bConverged<< "\t"<< "iterations: " <<m_iterationsNeeded <<" / "<<  m_Settings.m_MaxIter<< std::endl;);
            #endif
            break;
        }
    }


}

template< typename TInclusionSolverConfig >
void InclusionSolverCONoG<TInclusionSolverConfig>::sorProxOverAllNodes() {

    // Move over all nodes, and do a sor prox step
    m_ContactGraph.applyNodeVisitor(*m_pSorProxStepNodeVisitor);
    // Move over all nodes, end of Sor Prox

    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
    bool converged;
    if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InVelocity) {
        typename ContactGraphType::BodyToContactsListIteratorType it;
        //std::cout << "Bodies: " << m_ContactGraph.m_SimBodyToContactsList.size() << std::endl;
        for(it=m_ContactGraph.m_SimBodyToContactsList.begin(); it !=m_ContactGraph.m_SimBodyToContactsList.end(); it++) {
            if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                //std::cout << "before Criteria"<<std::endl;
                //std::cout <<"new "<< it->first->m_pSolverData->m_uBuffer.m_front.transpose() << std::endl;
                //std::cout <<"old "<< it->first->m_pSolverData->m_uBuffer.m_back.transpose() << std::endl;
                converged = Numerics::cancelCriteriaValue(it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
                            it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
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
            it->first->m_pSolverData->m_uBuffer.m_back = it->first->m_pSolverData->m_uBuffer.m_front;
        }
    }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings<LayoutConfigType>::InEnergyVelocity){
        typename ContactGraphType::BodyToContactsListIteratorType it;
        for(it=m_ContactGraph.m_SimBodyToContactsList.begin(); it !=m_ContactGraph.m_SimBodyToContactsList.end(); it++) {
            if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {

                converged = Numerics::cancelCriteriaMatrixNorm( it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
                                                                it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
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
            it->first->m_pSolverData->m_uBuffer.m_back = it->first->m_pSolverData->m_uBuffer.m_front;
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
    << m_PercussionPool.getPoolSize();
    return s.str();
}
#endif

#include "InclusionSolverCONoGMPI.hpp"

//#include "InclusionCommunicator.hpp"
// those two include each other (forwarding)
//#include "ContactGraphMPI.hpp"


#include "ConfigureFile.hpp"

#include "MatrixHelpers.hpp"
#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif




InclusionSolverCONoG::InclusionSolverCONoG(
    boost::shared_ptr< BodyCommunicator >  pBodyComm,
    boost::shared_ptr< CollisionSolverType >  pCollisionSolver,
    boost::shared_ptr< DynamicsSystemType > pDynSys,
    boost::shared_ptr< ProcessCommunicatorType > pProcCom
):
    m_SimBodies(pDynSys->m_SimBodies),
    m_Bodies(pDynSys->m_Bodies),
    m_pDynSys(pDynSys),
    m_pCollisionSolver(pCollisionSolver),
    m_pBodyComm(pBodyComm),
    m_pProcComm(pProcCom),
    m_pProcInfo(m_pProcComm->getProcInfo()),
    m_nbRanks(m_pProcComm->getProcInfo()->getProcTopo()->getNeighbourRanks()) {

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }


    m_timeProx = 0;
    m_proxIterationTime = 0;
    m_bUsedGPU = false;
    m_isFinite = 0;
    m_nContacts = 0;
    m_globalIterationCounter =0;
    m_bConverged = true;


    //Make a new Sor Prox Visitor (takes references from these class member)
    m_pSorProxStepNodeVisitor = new SorProxStepNodeVisitor<ContactGraphType>(m_Settings,m_bConverged,m_globalIterationCounter);
    m_pSorProxInitNodeVisitor = new SorProxInitNodeVisitor<ContactGraphType>();
    m_pSorProxStepSplitNodeVisitor = new SorProxStepSplitNodeVisitor<ContactGraphType>(m_Settings,m_bConverged,m_globalIterationCounter);

    m_pInclusionComm = boost::shared_ptr<InclusionCommunicatorType >( new InclusionCommunicatorType(pBodyComm, m_pDynSys,  m_pProcComm));
    m_pContactGraph  = boost::shared_ptr<ContactGraphType>( new ContactGraphType(pDynSys));

    m_pContactGraph->setInclusionCommunicator( m_pInclusionComm );
    m_pInclusionComm->setContactGraph( m_pContactGraph );

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->m_ContactDelegateList.addContactDelegate(
        ContactDelegateList::ContactDelegate::from_method< ContactGraphType,  &ContactGraphType::addNode>(m_pContactGraph.get())
    );


    m_nodesLocal     = m_pContactGraph->getLocalNodeListRef();
    m_nodesRemote    = m_pContactGraph->getRemoteNodeListRef();
    m_nodesSplitBody = m_pContactGraph->getSplitBodyNodeListRef();

}


InclusionSolverCONoG::~InclusionSolverCONoG() {
    delete m_pSorProxStepNodeVisitor;
    delete m_pSorProxInitNodeVisitor;
    delete m_pSorProxStepSplitNodeVisitor;
}


void InclusionSolverCONoG::initializeLog( Logging::Log * pSolverLog,  boost::filesystem::path folder_path ) {

    m_pSolverLog = pSolverLog;

    m_pContactGraph->setLog(m_pSolverLog);
    m_pSorProxStepNodeVisitor->setLog(m_pSolverLog);
    m_pSorProxInitNodeVisitor->setLog(m_pSolverLog);
    m_pSorProxStepSplitNodeVisitor->setLog(m_pSolverLog);


#if HAVE_CUDA_SUPPORT == 1

#endif
}


void InclusionSolverCONoG::reset() {

    m_pDynSys->getSettings(m_Settings);

    resetForNextIter();

#if HAVE_CUDA_SUPPORT == 1
    LOG(m_pSimulationLog, "---> Try to set GPU Device : "<< m_Settings.m_UseGPUDeviceId << std::endl;);

    CHECK_CUDA(cudaSetDevice(m_Settings.m_UseGPUDeviceId));
    cudaDeviceProp props;
    CHECK_CUDA(cudaGetDeviceProperties(&props,m_Settings.m_UseGPUDeviceId));

    LOG(m_pSimulationLog,  "---> Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << std::endl;);
#endif




}


void InclusionSolverCONoG::resetForNextIter() {

    m_nContacts = 0;
    m_nSplitBodyNodes = 0;
    m_globalIterationCounter = 0;

    m_bConverged = true;

    m_pContactGraph->clearGraph();

    m_pInclusionComm->clearNeighbourMap();
    m_pInclusionComm->setSettings(m_Settings);
}



void InclusionSolverCONoG::solveInclusionProblem(PREC currentSimulationTime) {

#if CoutLevelSolver>1
    LOG(m_pSolverLog,  "---> solveInclusionProblem(): "<< std::endl;);
#endif



    // Standart values
    m_currentSimulationTime = currentSimulationTime;
    m_globalIterationCounter = 0;
    m_bConverged = false; // Set true later, if one node is not converged then 0! and we do one more loop
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;


    // First communicate all remote bodies, which have contacts, to the owner, all processes are involved
#if CoutLevelSolverWhenContact>1
    LOG(m_pSolverLog,  "MPI> Communicate Remote Contacts (splitted bodies)" << std::endl; );
#endif
    m_pInclusionComm->communicateRemoteContacts(m_currentSimulationTime);

    // All detected contacts in ths process
    m_nLocalNodes  = m_nodesLocal.size();
    m_nRemoteNodes = m_nodesRemote.size();
    m_nContacts = m_nContactsLocal + m_nContactsRemote;
    m_nSplitBodyNodes = m_nodesSplitBody.size();

    // Integrate all local bodies to u_e
    // u_E = u_S + M^⁻1 * h * deltaT

    if(m_nContactsLocal == 0 && m_nSplitBodyNodes==0 && m_nContactsRemote==0) {
        // Nothing to solve
        integrateAllBodyVelocities();

    } else {

        // Solve Inclusion

#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog,
            "---> Nodes Local: "<< nodesLocal.size() <<std::endl<<
            "---> Nodes Remote: "<< nodesRemote.size() <<std::endl<<
            "---> Nodes SplitBodies: "<< nodesSplitBody.size() <<std::endl;
           );
#endif



        // =============================================================================================================
        if( m_Settings.m_eMethod == InclusionSolverSettingsType::SOR) {

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

        } else if(m_Settings.m_eMethod == InclusionSolverSettingsType::JOR) {
            ASSERTMSG(false,"Jor Algorithm has not been implemented yet");
        }

        if(m_Settings.m_bIsFiniteCheck) {
            // TODO CHECK IF finite!
#if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog,  "---> Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
#endif
        }


#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog,  "---> Prox Iterations needed: "<< m_globalIterationCounter <<std::endl;);
#endif

#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog,  "---> Finalize Prox " <<std::endl; );
#endif

        finalizeSorProx();

    }



}



void InclusionSolverCONoG::doJorProx() {
    ASSERTMSG(false,"InclusionSolverCONoG:: JOR Prox iteration not implemented!");
}


void InclusionSolverCONoG::integrateAllBodyVelocities() {


    for( auto bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
    }



}



void InclusionSolverCONoG::doSorProx() {

#if CoutLevelSolverWhenContact>2
    LOG(m_pSolverLog, " u_e = [ ");
    for(auto it = m_SimBodies.begin(); it != m_SimBodies.end(); it++) {
        LOG(m_pSolverLog, "\t uBack: " << (*it)->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl);
        LOG(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
    }
    LOG(m_pSolverLog, " ]" << std::endl);
#endif

    // General stupid Prox- Iteration
    while(true) {

        m_bConverged = true;
#if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog,"---> Next iteration: "<< m_globalIterationCounter << std::endl);
#endif
        sorProxOverAllNodes(); // Do one global Sor Prox Iteration

#if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, " u_e = [ ");
        for(auto it = m_SimBodies.begin(); it != m_SimBodies.end(); it++) {
            LOG(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
        }
        LOG(m_pSolverLog, " ]" << std::endl);
#endif


        m_globalIterationCounter++;

        if ( (m_bConverged == true || m_globalIterationCounter >= m_Settings.m_MaxIter) && m_globalIterationCounter >= m_Settings.m_MinIter) {
#if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog, "---> converged = "<<m_bConverged<< "\t"<< "iterations: " <<m_globalIterationCounter <<" / "<<  m_Settings.m_MaxIter<< std::endl;);
#endif
            break;
        }
    }


}



void InclusionSolverCONoG::initContactGraphForIteration(PREC alpha) {



    m_pSorProxInitNodeVisitor->setParams(alpha);

    // Init local nodes
    if(m_nLocalNodes){
        m_pContactGraph->applyNodeVisitorLocal(*m_pSorProxInitNodeVisitor);
    }
    // Init Remote nodes
    if(m_nRemoteNodes){
        m_pContactGraph->applyNodeVisitorRemote(*m_pSorProxInitNodeVisitor);
    }

    // Init SplitBodyNodes has already been done during communication!
    //m_pContactGraph->applyNodeVisitorSplitBody(*m_pSorProxInitSplitBodyNodeVisitor);


    // Set the initial u_0 for the prox iteration in the velocities for LOCAL BODIES!
    for( auto bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        // add u_s + M^⁻1*h*deltaT ,  all contact forces initial values have already been applied!
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back +
                (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
    }

    // Set the initial u_0 for the prox iteration for all REMOTE BODIES WITH CONTACTS
    auto & remotesWithContacts = m_pContactGraph->getRemoteBodiesWithContactsListRef();
    for( auto bodyIt = remotesWithContacts.begin(); bodyIt != remotesWithContacts.end(); bodyIt++) {
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back +
                (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
    }



}


void InclusionSolverCONoG::sorProxOverAllNodes() {

    bool doConvergenceCheck;

    // if only local nodes then we do always a convergence check after each global iteration
    if(m_nLocalNodes>=0 && m_nRemoteNodes == 0 && m_nSplitBodyNodes == 0){
        doConvergenceCheck = true;
    }else{
        doConvergenceCheck = m_globalIterationCounter % (m_Settings.m_convergenceCheckRatio*m_Settings.m_splitNodeUpdateRatio)  == 0
                             && m_globalIterationCounter >= m_Settings.m_MinIter;
    }

    // cache the velocities if convergence check should be done
    if( doConvergenceCheck ) {

        if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocity ||
                m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyVelocity) {
            //For all remote and local bodies
            auto & localWithContacts   = m_pContactGraph->getLocalBodiesWithContactsListRef();
            auto & remotesWithContacts = m_pContactGraph->getRemoteBodiesWithContactsListRef();

            for( auto bodyIt = localWithContacts.begin(); bodyIt != localWithContacts.end(); bodyIt++) {
                (*bodyIt)->m_pSolverData->m_uBuffer.m_back = (*bodyIt)->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
            }
            for( auto bodyIt = remotesWithContacts.begin(); bodyIt != remotesWithContacts.end(); bodyIt++) {
                (*bodyIt)->m_pSolverData->m_uBuffer.m_back = (*bodyIt)->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
            }
        }
    }

    // Move over all local nodes, and do a sor prox step
    if(m_nLocalNodes){
        m_pContactGraph->applyNodeVisitorLocal(*m_pSorProxStepNodeVisitor);
    }
    // Move over all remote nodes, and do a sor prox step
    if(m_nRemoteNodes){
        m_pContactGraph->applyNodeVisitorRemote(*m_pSorProxStepNodeVisitor);
    }


    // Do this only after a certain numer of iterations!
    if(m_nSplitBodyNodes){
        if(m_globalIterationCounter % m_Settings.m_splitNodeUpdateRatio == 0 ) {
            // Communicate all remote velocities to neighbour (if nSplitBodies != 0 || remoteNodes != 0)
            m_pInclusionComm->communicateSplitBodyUpdate(m_globalIterationCounter);

            // Safety test if all updates have been received!
            SplitNodeCheckUpdateVisitor<ContactGraphType> v;
            m_pContactGraph->applyNodeVisitorSplitBody(v);
            // Move over all split body nodes and solve the billateral constraint directly
            //(if nSplitBodies != 0)
            m_pContactGraph->applyNodeVisitorSplitBody(*m_pSorProxStepSplitNodeVisitor);
            // Communicate all local solved split body velocities
            //(if nSplitBodies != 0 || remoteNodes != 0)
            m_pInclusionComm->communicateSplitBodySolution(m_globalIterationCounter);
        }
    }


    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
    // (m_Settings.m_convergenceCheckRatio*m_Settings.m_splitNodeUpdateRatio) = local iterations per convergence checks
    if(doConvergenceCheck) {

        bool converged;
        auto & localWithContacts   = m_pContactGraph->getLocalBodiesWithContactsListRef();
        auto & remotesWithContacts = m_pContactGraph->getRemoteBodiesWithContactsListRef();

        if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocity) {
            //Local Bodies

            for(auto it = localWithContacts.begin(); it !=localWithContacts.end(); it++) {
                //std::cout << "before Criteria"<<std::endl;//std::cout <<"new "<< (*it)->first->m_pSolverData->m_uBuffer.m_front.transpose() << std::endl; //std::cout <<"old "<< (*it)->first->m_pSolverData->m_uBuffer.m_back.transpose() << std::endl;
                converged = Numerics::cancelCriteriaValue(  (*it)->m_pSolverData->m_uBuffer.m_back,
                                                            (*it)->m_pSolverData->m_uBuffer.m_front,
                                                            m_Settings.m_AbsTol,
                                                            m_Settings.m_RelTol);
                //std::cout << "after Criteria"<<std::endl;
                if(!converged) {
                    m_bConverged=false;
                    break; // every body needs to be converged
                }
            }
            // Remote Bodies
            if( m_bConverged ) {
                for(auto it=remotesWithContacts.begin(); it !=remotesWithContacts.end(); it++) {
                    converged = Numerics::cancelCriteriaValue(  (*it)->m_pSolverData->m_uBuffer.m_back, (*it)->m_pSolverData->m_uBuffer.m_front,m_Settings.m_AbsTol,m_Settings.m_RelTol);

                    if(!converged) {
                        m_bConverged=false;
                        break;
                    }
                }
            }

        } else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyVelocity) {

            for(auto it=localWithContacts.begin(); it!=localWithContacts.end(); it++) {
                converged = Numerics::cancelCriteriaMatrixNorm( (*it)->m_pSolverData->m_uBuffer.m_back,
                                                                (*it)->m_pSolverData->m_uBuffer.m_front,
                                                                (*it)->m_MassMatrix_diag,
                                                                m_Settings.m_AbsTol,
                                                                m_Settings.m_RelTol);
                if(!converged) {
                    m_bConverged=false;
                    break;
                }
            }

           // Remote Bodies
            if( m_bConverged ) {
                for(auto it = remotesWithContacts.begin(); it !=remotesWithContacts.end(); it++) {

                    converged = Numerics::cancelCriteriaMatrixNorm( (*it)->m_pSolverData->m_uBuffer.m_back,(*it)->m_pSolverData->m_uBuffer.m_front,(*it)->m_MassMatrix_diag,m_Settings.m_AbsTol,m_Settings.m_RelTol);
                    if(!converged) { m_bConverged=false; break; }
                }
            }

        }


        if(m_nLocalNodes>=0 && m_nRemoteNodes == 0 && m_nSplitBodyNodes == 0){

        }else{
             //Communicates our converged flags and sets it to false if some neighbours are not convgered
             m_bConverged =  m_pInclusionComm->communicateConvergence(m_bConverged);
        }



    } // end doConvergence
    else{
        m_bConverged = false;
    }
}


void InclusionSolverCONoG::finalizeSorProx() {

    // Set all weightings of remote and local bodies back to the original!
#if CoutLevelSolverWhenContact>0
    LOG(m_pSolverLog,  "---> Reset All Weigths" <<std::endl;);
#endif

    m_pInclusionComm->resetAllWeightings();
}

std::string  InclusionSolverCONoG::getIterationStats() {
    std::stringstream s;
    s
            << m_bUsedGPU<<"\t"
            << m_nContacts<<"\t"
            << m_nContactsLocal << "\t"
            << m_nContactsRemote << "\t"
            << m_nSplitBodyNodes << "\t"
            << m_globalIterationCounter<<"\t"
            << m_bConverged<<"\t"
            << m_isFinite<<"\t"
            << m_timeProx<<"\t"
            << m_proxIterationTime<<"\t"
            << m_pDynSys->m_CurrentStateEnergy;
    return s.str();
}

std::string InclusionSolverCONoG::getStatsHeader() {
    std::stringstream s;
    s
            << "GPUUsed"<<"\t"
            << "nContacts"<<"\t"
            << "nContactsLocal" << "\t"
            << "nContactsRemote" << "\t"
            << "nSplitBodyNodes" << "\t"
            << "nGlobalIterations"<<"\t"
            << "Converged"<<"\t"
            << "IsFinite"<<"\t"
            << "TotalTimeProx [s]"<<"\t"
            << "IterTimeProx [s]"<<"\t"
            << "TotalStateEnergy [J]";
    return s.str();
}

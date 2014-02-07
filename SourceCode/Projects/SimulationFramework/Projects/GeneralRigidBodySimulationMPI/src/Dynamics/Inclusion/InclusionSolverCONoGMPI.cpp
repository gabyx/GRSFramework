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
    m_nbRanks(m_pProcComm->getProcInfo()->getProcTopo()->getNeighbourRanks())
{

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

}


InclusionSolverCONoG::~InclusionSolverCONoG(){
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



void InclusionSolverCONoG::solveInclusionProblem() {

#if CoutLevelSolver>1
    LOG(m_pSolverLog,  "---> solveInclusionProblem(): "<< std::endl;);
#endif

    auto & nodesLocal = m_pContactGraph->getLocalNodeListRef();
    auto & nodesRemote = m_pContactGraph->getRemoteNodeListRef();
    auto & nodesSplitBody = m_pContactGraph->getSplitBodyNodeListRef();

    // Standart values
    m_globalIterationCounter = 0;
    m_bConverged = false; // Set true later, if one node is not converged then 0! and we do one more loop
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;


     // First communicate all remote bodies, which have contacts, to the owner
    #if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog,  "MPI> Communicate Remote Contacts (splitted bodies)" << std::endl; );
    #endif
    m_pInclusionComm->communicateRemoteContacts();

    // All detected contacts in ths process
    m_nContactsLocal = nodesLocal.size();
    m_nContactsRemote = nodesRemote.size();
    m_nContacts = m_nContactsLocal + m_nContactsRemote;
    m_nSplitBodyNodes = nodesSplitBody.size();

    // Integrate all local bodies to u_e
    // u_E = u_S + M^⁻1 * h * deltaT

   if(m_nContactsLocal == 0 && m_nSplitBodyNodes==0 && m_nContactsRemote==0){
        // Nothing to solve
        integrateAllBodyVelocities();
    }
    else{

    // Solve Inclusion

        #if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog,
                "---> Nodes Local: "<< nodesLocal.size() <<std::endl<<
                "---> Nodes Remote: "<< nodesRemote.size() <<std::endl<<
                "---> Nodes SplitBodies: "<< nodesSplitBody.size() <<std::endl;
            );
        #endif



        // =============================================================================================================
        if( m_Settings.m_eMethod == InclusionSolverSettings::SOR) {

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

        } else if(m_Settings.m_eMethod == InclusionSolverSettings::JOR) {
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
    }

#if CoutLevelSolverWhenContact>0
    LOG(m_pSolverLog,  "---> Finalize Prox " <<std::endl; );
#endif

    finalizeSorProx();

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



void InclusionSolverCONoG::initContactGraphForIteration(PREC alpha) {



    m_pSorProxInitNodeVisitor->setParams(alpha);





    // Init local nodes
    m_pContactGraph->applyNodeVisitorLocal(*m_pSorProxInitNodeVisitor);

    // Init Remote nodes
    m_pContactGraph->applyNodeVisitorRemote(*m_pSorProxInitNodeVisitor);

    // Init SplitBodyNodes has already been done during communication!
    //m_pContactGraph->applyNodeVisitorSplitBody(*m_pSorProxInitSplitBodyNodeVisitor);


    // Set the initial u_0 for the prox iteration in the velocities for LOCAL BODIES!
    // The ones which do not participate in the contact graph are already integrated
    for( auto bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        // add u_s + M^⁻1*h*deltaT ,  all contact forces initial values have already been applied!
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back +
                            (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
        (*bodyIt)->m_pSolverData->m_uBuffer.m_back = (*bodyIt)->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
    }

    // The initialization of the front velocity for all remote bodies taking part in a split body node
    // has already been done in the communication step before




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


void InclusionSolverCONoG::sorProxOverAllNodes() {

    // Move over all local nodes, and do a sor prox step
    m_pContactGraph->applyNodeVisitorLocal(*m_pSorProxStepNodeVisitor);
    // Move over all remote nodes, and do a sor prox step
    m_pContactGraph->applyNodeVisitorRemote(*m_pSorProxStepNodeVisitor);


    // Communicate all remote velocities to neighbour :-)
    m_pInclusionComm->communicateSplitBodyUpdate(m_globalIterationCounter);

    // Safety test if all updates have been received!
    SplitNodeCheckUpdateVisitor<ContactGraphType> v;
    m_pContactGraph->applyNodeVisitorSplitBody(v);
    // Move over all split body nodes and solve the billateral constraint directly
    m_pContactGraph->applyNodeVisitorSplitBody(*m_pSorProxStepSplitNodeVisitor);
    // Communicate all local solved split body velocities
    m_pInclusionComm->communicateSplitBodySolution(m_globalIterationCounter);


//    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
//    bool converged;
//    if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InVelocity) {
//        //std::cout << "Bodies: " << m_pContactGraph->m_SimBodyToContactsList.size() << std::endl;
//        for(auto it=m_pContactGraph->m_SimBodyToContactsList.begin(); it !=m_pContactGraph->m_SimBodyToContactsList.end(); it++) {
//            if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
//                //std::cout << "before Criteria"<<std::endl;
//                //std::cout <<"new "<< it->first->m_pSolverData->m_uBuffer.m_front.transpose() << std::endl;
//                //std::cout <<"old "<< it->first->m_pSolverData->m_uBuffer.m_back.transpose() << std::endl;
//                converged = Numerics::cancelCriteriaValue(it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
//                            it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
//                            m_Settings.m_AbsTol,
//                            m_Settings.m_RelTol);
//                //std::cout << "after Criteria"<<std::endl;
//                if(!converged) {
//                    m_bConverged=false;
//                }
//
//            } else {
//                m_bConverged=false;
//            }
//            // Do not switch Velocities (for next Sor Prox Iteration)
//            // Just fill back buffer with new values!
//            it->first->m_pSolverData->m_uBuffer.m_back = it->first->m_pSolverData->m_uBuffer.m_front;
//        }
//    }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InEnergyVelocity){
//        for(auto it=m_pContactGraph->m_SimBodyToContactsList.begin(); it !=m_pContactGraph->m_SimBodyToContactsList.end(); it++) {
//            if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
//
//                converged = Numerics::cancelCriteriaMatrixNorm( it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
//                                                                it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
//                                                                it->first->m_MassMatrix_diag,
//                                                                m_Settings.m_AbsTol,
//                                                                m_Settings.m_RelTol);
//                if(!converged) {
//                    m_bConverged=false;
//                }
//
//            } else {
//                m_bConverged=false;
//            }
//            // Do not switch Velocities (for next Sor Prox Iteration)
//            // Just fill back buffer with new values! for next global iteration
//            it->first->m_pSolverData->m_uBuffer.m_back = it->first->m_pSolverData->m_uBuffer.m_front;
//        }
//    }


}


void InclusionSolverCONoG::finalizeSorProx(){

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

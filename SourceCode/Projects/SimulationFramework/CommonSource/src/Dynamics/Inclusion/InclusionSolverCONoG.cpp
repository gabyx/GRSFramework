#include "InclusionSolverCONoG.hpp"

#include "MatrixHelpers.hpp"
#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"


InclusionSolverCONoG::InclusionSolverCONoG(boost::shared_ptr< CollisionSolverType >  pCollisionSolver,
                                           boost::shared_ptr<DynamicsSystemType > pDynSys):
    m_SimBodies(pDynSys->m_SimBodies),
    m_Bodies(pDynSys->m_Bodies),
    m_ContactGraph(&(pDynSys->m_ContactParameterMap)){

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }

    m_pCollisionSolver = pCollisionSolver;

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->m_ContactDelegateList.addContactDelegate(
        ContactDelegateList::ContactDelegate::from_method< ContactGraphType,  &ContactGraphType::addNode>(&m_ContactGraph)
    );

    m_nContacts = 0;

    m_globalIterationCounter =0;
    m_bConverged = true;
    m_pDynSys = pDynSys;

    //Make a new Sor Prox Visitor (takes references from these class member)
    m_pSorProxStepNodeVisitor = new SorProxStepNodeVisitor(m_Settings,m_bConverged,m_globalIterationCounter,&m_ContactGraph);
    m_pSorProxInitNodeVisitor = new SorProxInitNodeVisitor(m_Settings);
}


InclusionSolverCONoG::~InclusionSolverCONoG(){
    delete m_pSorProxStepNodeVisitor;
    delete m_pSorProxInitNodeVisitor;
}


void InclusionSolverCONoG::initializeLog( Logging::Log * pSolverLog,  boost::filesystem::path folder_path ) {

    m_pSolverLog = pSolverLog;
    m_pSorProxStepNodeVisitor->setLog(m_pSolverLog);
    m_pSorProxInitNodeVisitor->setLog(m_pSolverLog);

    // if we should
    #if OUTPUT_SIMDATAITERATION_FILE == 1
        boost::filesystem::path file = folder_path / "SimDataIteration.dat";
        LOG(m_pSimulationLog, "---> Open SimDataIteration file : "<< file<< std::endl;)
        m_iterationDataFile.open(file.string(), std::ios::trunc | std::ios::out);
    #endif

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

    #if OUTPUT_SIMDATAITERATION_FILE == 1
    LOG(m_pSimulationLog,  "---> Close IterationDataFile" << std::endl;);
    m_iterationDataFile.close();
    #endif

}


void InclusionSolverCONoG::resetForNextIter() {

    m_nContacts = 0;
    m_globalIterationCounter =0;

    m_bConverged = true;

    m_ContactGraph.clearGraph();
}



void InclusionSolverCONoG::solveInclusionProblem() {

#if CoutLevelSolver>1
    LOG(m_pSolverLog,  "---> solveInclusionProblem(): "<< std::endl;);
#endif


    // Iterate over all nodes set and assemble the matrices...
    typename ContactGraphType::NodeListType & nodes = m_ContactGraph.getNodeListRef();
    m_nContacts = (unsigned int)nodes.size();

    // Standart values
    m_globalIterationCounter = 0;
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
            LOG(m_pSolverLog,  "--->  nContacts: "<< m_nContacts <<std::endl;);
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
                LOG(m_pSolverLog,  "--->  Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
            #endif
        }

        //TODO update ContactPercussions
        #if USE_PERCUSSION_POOL == 1
            //updatePercussionPool(P_front);
        #endif

#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog,  "---> Prox Iterations needed: "<< m_globalIterationCounter <<std::endl;);
#endif
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



void InclusionSolverCONoG::initContactGraphForIteration(PREC alpha) {

    // Calculates b vector for all nodes, u_0, R_ii, ...
    m_pSorProxInitNodeVisitor->setParams(alpha);
    m_pSorProxStepNodeVisitor->setParams(alpha);

    m_ContactGraph.applyNodeVisitor(*m_pSorProxInitNodeVisitor);

    // Integrate all bodies!
    for( auto bodyIt = m_SimBodies.begin(); bodyIt != m_SimBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        // add u_s + M^⁻1*h*deltaT ,  all contact forces initial values have already been applied!
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()  *  (*bodyIt)->m_h_term * m_Settings.m_deltaT;
        (*bodyIt)->m_pSolverData->m_uBuffer.m_back = (*bodyIt)->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
    }
}


void InclusionSolverCONoG::doSorProx() {

    #if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, "---> u_e = [ ");
        for(auto it = m_SimBodies.begin(); it != m_SimBodies.end(); it++) {
            LOG(m_pSolverLog, "\t uBack: " << (*it)->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl);
            LOG(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
        }
        LOG(m_pSolverLog, " ]" << std::endl);
    #endif

    // General stupid Prox- Iteration
    while(true) {

        // Set global flag to true, if it is not converged we set it to false
        m_bConverged = true;

        #if CoutLevelSolverWhenContact>1
            LOG(m_pSolverLog,"---> Next iteration: "<< m_globalIterationCounter << std::endl);
        #endif

        // Do one Sor Prox Iteration
        sorProxOverAllNodes();

        #if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, "---> u_e = [ ");
        for(auto it = m_SimBodies.begin(); it != m_SimBodies.end(); it++) {
            LOG(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
        }
        LOG(m_pSolverLog, " ]" << std::endl);
        #endif


        m_globalIterationCounter++;

        if ( (m_bConverged == true || m_globalIterationCounter >= m_Settings.m_MaxIter)
             && m_globalIterationCounter >= m_Settings.m_MinIter) {
            #if CoutLevelSolverWhenContact>0
                LOG(m_pSolverLog, "---> converged = "<<m_bConverged<< "\t"<< "iterations: " <<m_globalIterationCounter <<" / "<<  m_Settings.m_MaxIter<< std::endl;);
            #endif
            break;
        }

        #if OUTPUT_SIMDATAITERATION_FILE == 1
            if(m_bConverged == false && m_globalIterationCounter >= m_Settings.m_MinIter){
                m_iterationDataFile << m_maxResidual << "\t";
            }
        #endif // OUTPUT_SIMDATAITERATION_FILE
    }


    #if OUTPUT_SIMDATAITERATION_FILE == 1
            if(m_bConverged == false && m_globalIterationCounter >= m_Settings.m_MinIter){
                m_iterationDataFile << std::endl;
            }
    #endif // OUTPUT_SIMDATAITERATION_FILE

}


void InclusionSolverCONoG::sorProxOverAllNodes() {

    m_maxResidual  = 0;
    m_pSorProxStepNodeVisitor->m_maxResidual = 0;

    // Move over all nodes, and do a sor prox step
    m_ContactGraph.applyNodeVisitor(*m_pSorProxStepNodeVisitor);
    m_maxResidual = m_pSorProxStepNodeVisitor->m_maxResidual;
    // Move over all nodes, end of Sor Prox

    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
    bool converged;
    PREC residual;

    if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocity) {

        //std::cout << "Bodies: " << m_ContactGraph.m_simBodiesToContactsList.size() << std::endl;
        for(auto it=m_ContactGraph.m_simBodiesToContactsList.begin(); it !=m_ContactGraph.m_simBodiesToContactsList.end(); it++) {
            if(m_globalIterationCounter >= m_Settings.m_MinIter && (m_bConverged || m_Settings.m_bComputeResidual) )  {

                converged = Numerics::cancelCriteriaValue(  it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
                                                            it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
                                                            m_Settings.m_AbsTol,
                                                            m_Settings.m_RelTol,
                                                            residual
                                                            );

                //std::cout << "after Criteria"<<std::endl;
                m_maxResidual = std::max(residual,m_maxResidual);
                if(!converged) {m_bConverged=false;}

            } else {
                m_bConverged=false;
            }
            // Do not switch Velocities (for next Sor Prox Iteration)
            // Just fill back buffer with new values!
            it->first->m_pSolverData->m_uBuffer.m_back = it->first->m_pSolverData->m_uBuffer.m_front;
        }

    }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyVelocity){

        for(auto it=m_ContactGraph.m_simBodiesToContactsList.begin(); it !=m_ContactGraph.m_simBodiesToContactsList.end(); it++) {
            if(m_globalIterationCounter >= m_Settings.m_MinIter && (m_bConverged || m_Settings.m_bComputeResidual)) {

                converged = Numerics::cancelCriteriaMatrixNormSq( it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
                                                                it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
                                                                it->first->m_MassMatrix_diag,
                                                                m_Settings.m_AbsTol,
                                                                m_Settings.m_RelTol,
                                                                residual
                                                                );
                m_maxResidual = std::max(residual,m_maxResidual);
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




std::string  InclusionSolverCONoG::getIterationStats() {
    std::stringstream s;
    s
    << m_bUsedGPU<<"\t"
    << m_nContacts<<"\t"
    << m_globalIterationCounter<<"\t"
    << m_bConverged<<"\t"
    << m_isFinite<<"\t"
    << m_timeProx<<"\t"
    << m_proxIterationTime<<"\t"
    << m_pDynSys->m_currentTotEnergy<<"\t"
    << m_pDynSys->m_currentKinEnergy<<"\t"
    << m_pDynSys->m_currentRotKinEnergy<<"\t"
    << m_pDynSys->m_currentSpinNorm;
    return s.str();
}

std::string InclusionSolverCONoG::getStatsHeader() {
    return std::string("GPUUsed\tnContacts\tnGlobalIterations\tConverged\tIsFinite\tTotalTimeProx [s]\tIterTimeProx [s]\tTotalStateEnergy [J]\tTotalKinEnergy [J]\tTotalRotKinEnergy [J]\tTotalSpinNorm [Nms]");
}

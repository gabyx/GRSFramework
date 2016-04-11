// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/dynamics/inclusion/InclusionSolverCONoGMPI.hpp"

//#include "GRSF/dynamics/inclusion/InclusionCommunicator.hpp"
// those two include each other (forwarding)
//#include "GRSF/dynamics/inclusion/ContactGraphMPI.hpp"


#include "GRSF/dynamics/general/MatrixHelpers.hpp"
#include "GRSF/dynamics/general/VectorToSkewMatrix.hpp"
#include "GRSF/dynamics/inclusion/ProxFunctions.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif




InclusionSolverCONoGMPI::InclusionSolverCONoGMPI(
    std::shared_ptr< BodyCommunicator >  pBodyComm,
    std::shared_ptr< CollisionSolverType >  pCollisionSolver,
    std::shared_ptr< DynamicsSystemType > pDynSys,
    std::shared_ptr< ProcessCommunicatorType > pProcComm
):
    m_pDynSys(pDynSys),
    m_pCollisionSolver(pCollisionSolver),
    m_pBodyComm(pBodyComm),
    m_pProcComm(pProcComm),
    m_simBodies(pDynSys->m_simBodies),
    m_staticBodies(pDynSys->m_staticBodies){

    if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
    } else {
        GRSF_ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }


    m_timeProx = 0;
    m_proxIterationTime = 0;
    m_bUsedGPU = false;
    m_isFinite = 0;
    m_nContacts = 0;
    m_globalIterationCounter =0;
    m_bConverged = true;

    // TODO, changing this order results in a not seen NodeDataType by compiler...
    m_pContactGraph  = new ContactGraphType(pDynSys);
    m_pInclusionComm =  new InclusionCommunicatorType(pBodyComm, m_pDynSys,  m_pProcComm);


    m_pContactGraph->setInclusionCommunicator( m_pInclusionComm );
    m_pInclusionComm->setContactGraph( m_pContactGraph );

}


InclusionSolverCONoGMPI::~InclusionSolverCONoGMPI() {
    if(m_pContactSorProxStepNodeVisitor != nullptr ){ delete m_pContactSorProxStepNodeVisitor;}
    if(m_pFullSorProxStepNodeVisitor != nullptr ){ delete m_pFullSorProxStepNodeVisitor;}
    if(m_pNormalSorProxStepNodeVisitor != nullptr ){ delete m_pNormalSorProxStepNodeVisitor;}
    if(m_pTangentialSorProxStepNodeVisitor != nullptr ){ delete m_pTangentialSorProxStepNodeVisitor;}
    if(m_pSorProxInitNodeVisitor != nullptr ){ delete m_pSorProxInitNodeVisitor;}
    if(m_pSorProxStepSplitNodeVisitor != nullptr ){ delete m_pSorProxStepSplitNodeVisitor;}

    delete m_pContactGraph;
    delete m_pInclusionComm;
}


void InclusionSolverCONoGMPI::initializeLog( Logging::Log * pSolverLog,  boost::filesystem::path folder_path ) {

    m_pSolverLog = pSolverLog;

    m_pContactGraph->setLog(m_pSolverLog);

    switch(m_settings.m_eMethod){
        case InclusionSolverSettingsType::Method::SOR_NORMAL_TANGENTIAL:
            m_pNormalSorProxStepNodeVisitor->setLog(m_pSolverLog);
            m_pTangentialSorProxStepNodeVisitor->setLog(m_pSolverLog);
            break;
        case InclusionSolverSettingsType::Method::SOR_FULL:
            m_pFullSorProxStepNodeVisitor->setLog(m_pSolverLog);
            break;
        case InclusionSolverSettingsType::Method::SOR_CONTACT:
            m_pContactSorProxStepNodeVisitor->setLog(m_pSolverLog);
            break;
        default:
            GRSF_ERRORMSG("InclusionSolverSettings::Method" << m_settings.m_eMethod << "not implemented");
    }

    m_pSorProxInitNodeVisitor->setLog(m_pSolverLog);

    m_pSorProxStepSplitNodeVisitor->setLog(m_pSolverLog);


#if HAVE_CUDA_SUPPORT == 1

#endif
}

void InclusionSolverCONoGMPI::resetTopology(){
      m_pInclusionComm->resetTopology();
      resetForNextTimestep();
}


void InclusionSolverCONoGMPI::reset() {

    m_settings = m_pDynSys->getSettingsInclusionSolver();

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->addContactDelegate(
        CollisionSolverType::ContactDelegateType::from_method< ContactGraphType,  &ContactGraphType::addNode<false> >(m_pContactGraph)
    );


    //Make a new Sor Prox Visitor (takes references from these class member)
    if(m_pContactSorProxStepNodeVisitor != nullptr ){ delete m_pContactSorProxStepNodeVisitor; m_pContactSorProxStepNodeVisitor = nullptr;}
    if(m_pFullSorProxStepNodeVisitor != nullptr ){ delete m_pFullSorProxStepNodeVisitor; m_pFullSorProxStepNodeVisitor = nullptr;}
    if(m_pNormalSorProxStepNodeVisitor != nullptr ){ delete m_pNormalSorProxStepNodeVisitor; m_pNormalSorProxStepNodeVisitor = nullptr;}
    if(m_pTangentialSorProxStepNodeVisitor != nullptr ){ delete m_pTangentialSorProxStepNodeVisitor; m_pTangentialSorProxStepNodeVisitor = nullptr;}
    if(m_pSorProxInitNodeVisitor != nullptr ){ delete m_pSorProxInitNodeVisitor; m_pSorProxInitNodeVisitor = nullptr;}
    if(m_pSorProxStepSplitNodeVisitor != nullptr ){ delete m_pSorProxStepSplitNodeVisitor; m_pSorProxStepSplitNodeVisitor = nullptr;}

    if(m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_CONTACT ){

         if(m_settings.m_eSubMethodUCF == InclusionSolverSettingsType::SubMethodUCF::UCF_AC){
            LOG(m_pSimulationLog, "---> Initialize ContactSorProxVisitor Alart Curnier "<<  std::endl;);
         }else if(m_settings.m_eSubMethodUCF == InclusionSolverSettingsType::SubMethodUCF::UCF_DS){
            LOG(m_pSimulationLog, "---> Initialize ContactSorProxVisitor De Saxé"<<  std::endl;);
         }
         m_pContactSorProxStepNodeVisitor = new ContactSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,m_pContactGraph);

    }else if( m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_FULL ){
         LOG(m_pSimulationLog, "---> Initialize FullSorProxVisitor Alart Curnier"<<  std::endl;);
         m_pFullSorProxStepNodeVisitor = new FullSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,m_pContactGraph);
    }else if(m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_NORMAL_TANGENTIAL) {
         m_pContactSorProxStepNodeVisitor = nullptr;
         m_pNormalSorProxStepNodeVisitor     = new NormalSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,m_pContactGraph);
         m_pTangentialSorProxStepNodeVisitor = new TangentialSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,m_pContactGraph);

    }else{
        GRSF_ERRORMSG("InclusionSolverSettings::Method" << m_settings.m_eMethod << "not implemented");
    }

    m_pSorProxInitNodeVisitor = new SorProxInitNodeVisitor<ContactGraphType>(m_settings);
    m_pSorProxStepSplitNodeVisitor = new SorProxStepSplitNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter);

    m_pInclusionComm->setSettings(m_settings);

    resetForNextTimestep();

    resetTopology();

    // Reserve Contacts (nodes and edges)
    m_pContactGraph->reserveNodes(m_settings.m_reserveContacts, m_settings.m_reserveSplitNodes);

}

void InclusionSolverCONoGMPI::resetForNextTimestep() {

    m_nContacts = 0;
    m_nSplitBodyNodes = 0;
    m_globalIterationCounter = 0;
    m_bConverged = true;

    m_avgIterTimeProxLocalRemoteNodes = 0;
    m_avgIterTimeProxSplitNodes = 0;

    m_pContactGraph->clearGraph();

    m_pInclusionComm->resetForNextTimestep();
}



void InclusionSolverCONoGMPI::solveInclusionProblem(PREC currentSimulationTime) {

    LOGSLLEVEL1(m_pSolverLog,  "---> solveInclusionProblem(): "<< std::endl;);


    // Standart values
    m_currentSimulationTime = currentSimulationTime;
    m_globalIterationCounter = 0;
    m_bConverged = false; // Set true later, if one node is not converged then 0! and we do one more loop
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;


    // First communicate all remote bodies, which have contacts, to the owner, all processes are involved
    LOGSLLEVEL2_CONTACT(m_pSolverLog,  "MPI> Communicate Remote Contacts (splitted bodies)" << std::endl; );
    m_pInclusionComm->communicateRemoteContacts(m_currentSimulationTime);

    // All detected contacts in ths process
    m_nLocalNodes  = m_pContactGraph->getNLocalNodes();
    m_nRemoteNodes = m_pContactGraph->getNRemoteNodes();
    m_nContacts = m_nLocalNodes + m_nRemoteNodes;
    m_nSplitBodyNodes = m_pContactGraph->getNSplitBodyNodes();

        LOGSLLEVEL1_CONTACT(m_pSolverLog,
            "---> Nodes Local: "<< m_nLocalNodes <<std::endl<<
            "---> Nodes Remote: "<< m_nRemoteNodes <<std::endl<<
            "---> Nodes SplitBodies: "<< m_nSplitBodyNodes <<std::endl;
           );

    // Integrate all local bodies to u_e
    // u_E = u_S + M^⁻1 * h * deltaT

    if(m_pContactGraph->hasNoNodes()) {
        // Nothing to solve
        integrateAllBodyVelocities();

    } else {

        // Solve Inclusion
        // =============================================================================================================
        if( m_settings.m_eMethod == InclusionSolverSettingsType::SOR_CONTACT ||
            m_settings.m_eMethod == InclusionSolverSettingsType::SOR_FULL
             ) {
#ifdef MEASURE_TIME_PROX
            CPUTimer counter;
            counter.start();
#endif

            initContactGraphForIteration(m_settings.m_alphaSORProx);
            doSorProx();

#ifdef MEASURE_TIME_PROX
            m_timeProx = counter.elapsedSec();
#endif

        } else if(m_settings.m_eMethod == InclusionSolverSettingsType::JOR) {
            GRSF_ASSERTMSG(false,"Jor Algorithm has not been implemented yet");
        }else{
            GRSF_ASSERTMSG(false,"This algorithm has not been implemented yet");
        }

        if(m_settings.m_bIsFiniteCheck) {
            // TODO CHECK IF finite!
            LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
        }


        LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Prox Iterations needed: "<< m_globalIterationCounter <<std::endl;);
        LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Finalize Prox " <<std::endl; );

        finalizeSorProx();
    }


    LOGSLLEVEL1_CONTACT(m_pSolverLog,  "MPI> Reset All Communicators" <<std::endl;);

    m_pProcComm->deleteAllCommunicators();

}



void InclusionSolverCONoGMPI::doJorProx() {
    GRSF_ASSERTMSG(false,"InclusionSolverCONoGMPI:: JOR Prox iteration not implemented!");
}


void InclusionSolverCONoGMPI::integrateAllBodyVelocities() {


    for( auto bodyIt = m_simBodies.begin(); bodyIt != m_simBodies.end(); bodyIt++) {
        // All bodies also the ones not in the contact graph...
        (*bodyIt)->m_pSolverData->m_uBuffer.m_front += (*bodyIt)->m_pSolverData->m_uBuffer.m_back + (*bodyIt)->m_MassMatrixInv_diag.asDiagonal()
                                                        *(*bodyIt)->m_h_term * m_settings.m_deltaT;
    }



}



void InclusionSolverCONoGMPI::doSorProx() {

    LOGSLLEVEL3_CONTACT(m_pSolverLog, " u_e = [ ");
    for(auto it = m_simBodies.begin(); it != m_simBodies.end(); ++it) {
        LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t uBack: " << (*it)->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl);
        LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
    }
    LOGSLLEVEL3_CONTACT(m_pSolverLog, " ]" << std::endl);

    // General stupid Prox- Iteration
    while(true) {

        m_bConverged = true;

        LOGSLLEVEL2_CONTACT(m_pSolverLog,"---> Next iteration: "<< m_globalIterationCounter << std::endl);


        sorProxOverAllNodes(); // Do one global Sor Prox Iteration



        LOGSLLEVEL3_CONTACT(m_pSolverLog, " u_e = [ ");
        for(auto it = m_simBodies.begin(); it != m_simBodies.end(); ++it) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
        }
        LOGSLLEVEL3_CONTACT(m_pSolverLog, " ]" << std::endl);


        m_globalIterationCounter++;

        if ( (m_bConverged == true || m_globalIterationCounter >= m_settings.m_MaxIter) && m_globalIterationCounter >= m_settings.m_MinIter) {

            LOGSLLEVEL1_CONTACT(m_pSolverLog, "---> converged = "<<m_bConverged<< "\t"<< "iterations: " <<m_globalIterationCounter <<" / "<<  m_settings.m_MaxIter<< std::endl;);
            break;
        }
    }


}



void InclusionSolverCONoGMPI::initContactGraphForIteration(PREC alpha) {

    m_pSorProxInitNodeVisitor->setParams(alpha);

     switch(m_settings.m_eMethod){
        case InclusionSolverSettingsType::Method::SOR_NORMAL_TANGENTIAL:
            m_pNormalSorProxStepNodeVisitor->setParams(alpha);
            m_pTangentialSorProxStepNodeVisitor->setParams(alpha);
            break;
        case InclusionSolverSettingsType::Method::SOR_FULL:
            m_pFullSorProxStepNodeVisitor->setParams(alpha);
            break;
        case InclusionSolverSettingsType::Method::SOR_CONTACT:
            m_pContactSorProxStepNodeVisitor->setParams(alpha);
            break;
        default:
            GRSF_ERRORMSG(" Visitor method not defined for visitor: " << EnumConversion::toIntegral(m_settings.m_eMethod));
    }


    // Init local and remote nodes
    if(m_nLocalNodes || m_nRemoteNodes){
        m_pContactGraph->visitNormalNodes(*m_pSorProxInitNodeVisitor);
    }

    // Init SplitBodyNodes has already been done during communication!
    //m_pContactGraph->applyNodeVisitorSplitBody(*m_pSorProxInitSplitBodyNodeVisitor);


    // Set the initial u_0 for the prox iteration in the velocities for LOCAL BODIES!
    for( auto * body : m_simBodies) {
        // All bodies also the ones not in the contact graph...
        // add u_s + M^⁻1*h*deltaT ,  all contact forces initial values have already been applied!
        body->m_pSolverData->m_uBuffer.m_front += body->m_pSolverData->m_uBuffer.m_back +
                body->m_MassMatrixInv_diag.asDiagonal()  *  body->m_h_term * m_settings.m_deltaT;
    }

    // Set the initial u_0 for the prox iteration for all REMOTE BODIES WITH CONTACTS
    auto & remotesWithContacts = m_pContactGraph->getRemoteBodiesWithContactsListRef();
    for( auto * body : remotesWithContacts) {
        // m_front is set zero
        body->m_pSolverData->m_uBuffer.m_front += body->m_pSolverData->m_uBuffer.m_back +
                body->m_MassMatrixInv_diag.asDiagonal()  *  body->m_h_term * m_settings.m_deltaT;
    }



}


void InclusionSolverCONoGMPI::sorProxOverAllNodes() {

    bool doConvergenceCheck;
    // if only local nodes then we do always a convergence check after each global iteration
    if( m_pContactGraph->isUncoupled() ){
        doConvergenceCheck = true;
    }else{
        doConvergenceCheck = m_globalIterationCounter % (m_settings.m_convergenceCheckRatio*m_settings.m_splitNodeUpdateRatio)  == 0
                             && m_globalIterationCounter >= m_settings.m_MinIter;
    }


    // cache the velocities if convergence check should be done
    if( doConvergenceCheck ) {

        if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocity ||
                m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyVelocity) {
            //For all remote and local bodies
            auto & localWithContacts   = m_pContactGraph->getLocalBodiesWithContactsListRef();
            auto & remotesWithContacts = m_pContactGraph->getRemoteBodiesWithContactsListRef();

            for( auto & body : localWithContacts) {
                body->m_pSolverData->m_uBuffer.m_back = body->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
            }
            for( auto & body : remotesWithContacts) {
                body->m_pSolverData->m_uBuffer.m_back = body->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
            }
        }else{
            GRSF_ERRORMSG("This cancelation criteria has not been implemented")
        }
    }

    {
    #ifdef MEASURE_TIME_PROX_DETAIL
        START_TIMER(start)
    #endif
    // Move over all local/remote nodes, and do a sor prox step
    if(m_nLocalNodes || m_nRemoteNodes){
       switch(m_settings.m_eMethod){
        case InclusionSolverSettingsType::Method::SOR_NORMAL_TANGENTIAL:
            //Iterate multiple times the normal direction before going to the tangential direction!
            m_pNormalSorProxStepNodeVisitor->setLastUpdate(false);
            for(int i = 0;i< m_settings.m_normalTangentialUpdateRatio ;++i){
                m_pContactGraph->visitNormalNodes(*m_pNormalSorProxStepNodeVisitor);
            }
            m_pNormalSorProxStepNodeVisitor->setLastUpdate(true);
            m_pContactGraph->visitNormalNodes(*m_pNormalSorProxStepNodeVisitor);

            m_pContactGraph->visitNormalNodes(*m_pTangentialSorProxStepNodeVisitor);
            break;
        case InclusionSolverSettingsType::Method::SOR_FULL:
            m_pContactGraph->visitNormalNodes(*m_pFullSorProxStepNodeVisitor);
            break;
        case InclusionSolverSettingsType::Method::SOR_CONTACT:
            m_pContactGraph->visitNormalNodes(*m_pContactSorProxStepNodeVisitor);
            break;
        default:
            GRSF_ERRORMSG(" Visitor method not defined for visitor: " << EnumConversion::toIntegral(m_settings.m_eMethod));
        }
    }


    #ifdef MEASURE_TIME_PROX_DETAIL
        STOP_TIMER_SEC(time,start);
        m_avgIterTimeProxLocalRemoteNodes = ( time + m_avgIterTimeProxLocalRemoteNodes*m_globalIterationCounter) / (m_globalIterationCounter+1);
    #endif
    }

    {
    #ifdef MEASURE_TIME_PROX_DETAIL
        START_TIMER(start)
    #endif
    // Do this only after a certain number of iterations!
    if(m_nSplitBodyNodes || m_nRemoteNodes ){
        if(m_globalIterationCounter % m_settings.m_splitNodeUpdateRatio == 0 ) {
            // Communicate all remote velocities to neighbour (if nSplitBodies != 0 || remoteNodes != 0)
            m_pInclusionComm->communicateSplitBodyUpdate(m_globalIterationCounter);


            // Safety test if all updates have been received!
            #ifdef SAFETY_CHECK_SPLITBODYUPDATE
                SplitNodeCheckUpdateVisitor<ContactGraphType> v;
                m_pContactGraph->visitSplitNodes(v);
            #endif
            // Move over all split body nodes and solve the billateral constraint directly
            //(if nSplitBodies != 0)
            m_pContactGraph->visitSplitNodes(*m_pSorProxStepSplitNodeVisitor);
            // Communicate all local solved split body velocities
            //(if nSplitBodies != 0 || remoteNodes != 0)
            m_pInclusionComm->communicateSplitBodySolution(m_globalIterationCounter);
        }
    }
    #ifdef MEASURE_TIME_PROX_DETAIL
        STOP_TIMER_SEC(time,start);
        m_avgIterTimeProxSplitNodes = ( time + m_avgIterTimeProxSplitNodes*m_globalIterationCounter) / (m_globalIterationCounter+1);
    #endif
    }

    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
    // (m_settings.m_convergenceCheckRatio*m_settings.m_splitNodeUpdateRatio) = local iterations per convergence checks
    if(doConvergenceCheck) {

        bool converged;
        auto & localWithContacts   = m_pContactGraph->getLocalBodiesWithContactsListRef();
        auto & remotesWithContacts = m_pContactGraph->getRemoteBodiesWithContactsListRef();

        if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocity) {

            //Local Bodies
            for(auto it = localWithContacts.begin(); it !=localWithContacts.end(); ++it) {
                //std::cout << "before Criteria"<<std::endl;//std::cout <<"new "<< (*it)->first->m_pSolverData->m_uBuffer.m_front.transpose() << std::endl; //std::cout <<"old "<< (*it)->first->m_pSolverData->m_uBuffer.m_back.transpose() << std::endl;
                converged = Numerics::cancelCriteriaValue(  (*it)->m_pSolverData->m_uBuffer.m_back,
                                                            (*it)->m_pSolverData->m_uBuffer.m_front,
                                                            m_settings.m_AbsTol,
                                                            m_settings.m_RelTol);
                //std::cout << "after Criteria"<<std::endl;
                if(!converged) {
                    m_bConverged=false;
                    break; // every body needs to be converged
                }
            }
            // Remote Bodies
            if( m_bConverged ) {
                for(auto it=remotesWithContacts.begin(); it !=remotesWithContacts.end(); ++it) {
                    converged = Numerics::cancelCriteriaValue(  (*it)->m_pSolverData->m_uBuffer.m_back, (*it)->m_pSolverData->m_uBuffer.m_front,m_settings.m_AbsTol,m_settings.m_RelTol);

                    if(!converged) {
                        m_bConverged=false;
                        break;
                    }
                }
            }

            LOGSLLEVEL2_CONTACT(m_pSolverLog, "---> Convergence criteria (InVelocity) converged: "<<  m_bConverged<< std::endl;);

        } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyVelocity) {
            //Local Bodies
            for(auto it=localWithContacts.begin(); it!=localWithContacts.end(); ++it) {
                converged = Numerics::cancelCriteriaMatrixNormSq( (*it)->m_pSolverData->m_uBuffer.m_back,
                                                                (*it)->m_pSolverData->m_uBuffer.m_front,
                                                                (*it)->m_MassMatrix_diag,
                                                                m_settings.m_AbsTol,
                                                                m_settings.m_RelTol);
                if(!converged) {
                    m_bConverged=false;
                    break;
                }
            }

           // Remote Bodies
            if( m_bConverged ) {
                for(auto it = remotesWithContacts.begin(); it !=remotesWithContacts.end(); ++it) {

                    converged = Numerics::cancelCriteriaMatrixNormSq( (*it)->m_pSolverData->m_uBuffer.m_back,(*it)->m_pSolverData->m_uBuffer.m_front,(*it)->m_MassMatrix_diag,m_settings.m_AbsTol,m_settings.m_RelTol);
                    if(!converged) { m_bConverged=false; break; }
                }
            }

            LOGSLLEVEL2_CONTACT(m_pSolverLog, "---> Convergence criteria (InEnergyVelocity) converged: "<<  m_bConverged<< std::endl;);

        }else{
            GRSF_ERRORMSG("This cancelation criteria has not been implemented")
        }

        // If local contact problem is uncoupled from other processes we do not check for convergence of other processes
        if( !m_pContactGraph->isUncoupled() ){
             //Communicates our converged flags and sets it to false if some neighbours are not convgered
             m_bConverged =  m_pInclusionComm->communicateConvergence(m_bConverged);
        }



    } // end doConvergence
    else{
        m_bConverged = false;
    }

    m_pContactGraph->resetAfterOneIteration(m_globalIterationCounter);


}


void InclusionSolverCONoGMPI::finalizeSorProx() {

    // Set all weightings of remote and local bodies back to the original!
    LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Reset All Weigths" <<std::endl;);

    m_pInclusionComm->resetAllWeightings();


}

std::string  InclusionSolverCONoGMPI::getIterationStats() {
    std::stringstream s;
    s << std::fixed
            << m_bUsedGPU<<"\t"
            << m_nContacts<<"\t"
            << m_nLocalNodes << "\t"
            << m_nRemoteNodes << "\t"
            << m_nSplitBodyNodes << "\t"
            << m_globalIterationCounter<<"\t"
            << m_bConverged<<"\t"
            << m_isFinite<<"\t"
            << m_timeProx<<"\t"
            << m_proxIterationTime<<"\t"
            << m_pDynSys->m_currentTotEnergy<<"\t"
            << m_pDynSys->m_currentPotEnergy<<"\t"
            << m_pDynSys->m_currentKinEnergy<<"\t"
            << m_pDynSys->m_currentRotKinEnergy<<"\t"
            << m_pDynSys->m_currentSpinNorm<<"\t"
            << m_avgIterTimeProxLocalRemoteNodes<<"\t"
            << m_avgIterTimeProxSplitNodes;
    return s.str();
}

std::string InclusionSolverCONoGMPI::getStatsHeader() {
    std::stringstream s;
    s << "GPUUsed\t"
         "nContacts\t"
         "nContactsLocal\t"
         "nContactsRemote\t"
         "nSplitBodyNodes\t"
         "nGlobalIterations\t"
         "Converged\t"
         "IsFinite\t"
         "TotalTimeProx [s]\t"
         "IterTimeProx [s]\t"
         "TotalStateEnergy [J]\t"
         "TotalPotEnergy [J]\t"
         "TotalKinEnergy [J]\t"
         "TotalRotKinEnergy [J]\t"
         "TotalSpinNorm [Nms]\t"
         "AvgTimeProxLocalRemoteNodes [s]\t"
         "AvgTimeProxSplitNodes [s]";
    return s.str();
}

#include "InclusionSolverCONoG.hpp"

#include "MatrixHelpers.hpp"
#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"


InclusionSolverCONoG::InclusionSolverCONoG(std::shared_ptr< CollisionSolverType >  pCollisionSolver,
                                           std::shared_ptr<DynamicsSystemType > pDynSys):
    m_simBodies(pDynSys->m_simBodies),
    m_staticBodies(pDynSys->m_staticBodies),
    m_contactGraph(&(pDynSys->m_ContactParameterMap)){

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }

    m_pCollisionSolver = pCollisionSolver;

    m_nContacts = 0;

    m_globalIterationCounter =0;
    m_bConverged = true;
    m_pDynSys = pDynSys;

    m_pSorProxStepNodeVisitor = nullptr;
    m_pNormalSorProxStepNodeVisitor = nullptr;
    m_pTangentialSorProxStepNodeVisitor = nullptr;
    m_pSorProxInitNodeVisitor = nullptr;

}


InclusionSolverCONoG::~InclusionSolverCONoG(){
    if(m_pSorProxStepNodeVisitor != nullptr ){ delete m_pSorProxStepNodeVisitor;}
    if(m_pNormalSorProxStepNodeVisitor != nullptr ){ delete m_pNormalSorProxStepNodeVisitor;}
    if(m_pTangentialSorProxStepNodeVisitor != nullptr ){ delete m_pTangentialSorProxStepNodeVisitor;}
    if(m_pSorProxInitNodeVisitor != nullptr ){ delete m_pSorProxInitNodeVisitor;}
}


void InclusionSolverCONoG::initializeLog( Logging::Log * pSolverLog,  boost::filesystem::path folder_path ) {

    m_pSolverLog = pSolverLog;

    if(m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_NORMAL_TANGENTIAL ){
        m_pNormalSorProxStepNodeVisitor->setLog(m_pSolverLog);
        m_pTangentialSorProxStepNodeVisitor->setLog(m_pSolverLog);
    }else{
        m_pSorProxStepNodeVisitor->setLog(m_pSolverLog);
    }

    m_pSorProxInitNodeVisitor->setLog(m_pSolverLog);

    // if we should
    #if OUTPUT_SIMDATAITERATION_FILE == 1
        boost::filesystem::path file = folder_path / "SimDataIteration.dat";
        LOG(m_pSimulationLog, "---> Open SimDataIteration file : "<< file<< std::endl;)
        m_iterationDataFile.open(file.string(), std::ios::trunc | std::ios::out);
    #endif

    #if HAVE_CUDA_SUPPORT == 1
        // initialize the cuda jor prox iteration module
        m_jorProxGPUModule.initializeLog(m_pSolverLog);
    #endif
}


void InclusionSolverCONoG::reset() {

    m_settings = m_pDynSys->getSettingsInclusionSolver();

    resetForNextIter();

#if HAVE_CUDA_SUPPORT == 1
    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    //to decide if we compute with the GPU (only JOR method) or we build the contact graph on the CPU to use the SOR method
    // Setups the node only partially!
    m_pCollisionSolver->addContactDelegate(
        CollisionSolverType::ContactDelegateType::from_method< ContactGraphType,  &ContactGraphType::addNode >(&m_contactGraph)
    );
    LOG(m_pSimulationLog,  "---> Registered ContactCallback in CollisionSolver for ContactGraph" << std::endl;);
#else
    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    // Setups the node fully!
    m_pCollisionSolver->addContactDelegate(
        CollisionSolverType::ContactDelegateType::from_method< ContactGraphType,  &ContactGraphType::addNode<true> >(&m_contactGraph)
    );
    LOG(m_pSimulationLog,  "---> Registered ContactCallback in CollisionSolver for ContactGraph" << std::endl;);
#endif


#if HAVE_CUDA_SUPPORT == 1
    // reset all the GPU jor porx iteration modules
    m_jorProxGPUModule.reset();
#endif

    #if OUTPUT_SIMDATAITERATION_FILE == 1
    LOG(m_pSimulationLog,  "---> Close IterationDataFile" << std::endl;);
    m_iterationDataFile.close();
    #endif


     //Make a new Sor Prox Visitor (takes references from these class member)
    if(m_pSorProxStepNodeVisitor != nullptr ){ delete m_pSorProxStepNodeVisitor; m_pSorProxStepNodeVisitor = nullptr;}
    if(m_pNormalSorProxStepNodeVisitor != nullptr ){ delete m_pNormalSorProxStepNodeVisitor; m_pNormalSorProxStepNodeVisitor = nullptr;}
    if(m_pTangentialSorProxStepNodeVisitor != nullptr ){ delete m_pTangentialSorProxStepNodeVisitor; m_pTangentialSorProxStepNodeVisitor = nullptr;}
    if(m_pSorProxInitNodeVisitor != nullptr ){ delete m_pSorProxInitNodeVisitor; m_pSorProxInitNodeVisitor = nullptr;}

    if(m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_CONTACT ){

         if(m_settings.m_eSubMethodUCF == InclusionSolverSettingsType::SubMethodUCF::UCF_AC){
            LOG(m_pSimulationLog, "---> Initialize ContactSorProxVisitor Alart Curnier "<<  std::endl;);
         }else if(m_settings.m_eSubMethodUCF == InclusionSolverSettingsType::SubMethodUCF::UCF_DS){

         }
         m_pSorProxStepNodeVisitor = new ContactSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,&m_contactGraph);

    }else if( m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_FULL ){
         LOG(m_pSimulationLog, "---> Initialize FullSorProxVisitor Alart Curnier"<<  std::endl;);
         m_pSorProxStepNodeVisitor = new FullSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,&m_contactGraph);
    }else if(m_settings.m_eMethod == InclusionSolverSettingsType::Method::SOR_NORMAL_TANGENTIAL) {
         m_pSorProxStepNodeVisitor = nullptr;
         m_pNormalSorProxStepNodeVisitor     = new NormalSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,&m_contactGraph);
         m_pTangentialSorProxStepNodeVisitor = new TangentialSorProxStepNodeVisitor<ContactGraphType>(m_settings,m_bConverged,m_globalIterationCounter,&m_contactGraph);

    }else{
        ERRORMSG("InclusionSolverSettings::Method" << m_settings.m_eMethod << "not implemendet");
    }

    m_pSorProxInitNodeVisitor = new SorProxInitNodeVisitor<ContactGraphType>(m_settings);

}


void InclusionSolverCONoG::resetForNextIter() {

    m_nContacts = 0;
    m_globalIterationCounter =0;

    m_bConverged = true;

    m_contactGraph.clearGraph();
}



void InclusionSolverCONoG::solveInclusionProblem() {


    LOGSLLEVEL2(m_pSolverLog,  "---> solveInclusionProblem(): "<< std::endl;);


    auto & contactDataList = m_pCollisionSolver->getCollisionSetRef();
    m_nContacts = (unsigned int)contactDataList.size();

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

        LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> nContacts: "<< m_nContacts <<std::endl;);


        // =============================================================================================================
        if( m_settings.m_eMethod == InclusionSolverSettingsType::SOR_CONTACT ||
            m_settings.m_eMethod == InclusionSolverSettingsType::SOR_FULL ||
            m_settings.m_eMethod == InclusionSolverSettingsType::SOR_NORMAL_TANGENTIAL
           ) {
#include "InclusionSolverSettings.hpp"
            #if MEASURE_TIME_PROX == 1
                CPUTimer counter;
                counter.start();
            #endif

            doSorProx();

            #if MEASURE_TIME_PROX == 1
                m_timeProx = counter.elapsedSec();
            #endif

        } else if(m_settings.m_eMethod == InclusionSolverSettingsType::JOR) {

            #if MEASURE_TIME_PROX == 1
                CPUTimer counter;
                counter.start();
            #endif


            doJorProx();

            #if MEASURE_TIME_PROX == 1
                m_timeProx = counter.elapsedSec();
            #endif
        }else{
            ASSERTMSG(false,"This algorithm has not been implemented yet");
        }

        if(m_settings.m_bIsFiniteCheck) {
            // TODO CHECK IF finite!
            LOGSLLEVEL1_CONTACT(m_pSolverLog,  "--->  Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
        }

        LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Prox Iterations needed: "<< m_globalIterationCounter <<std::endl;);
    }

}



void InclusionSolverCONoG::doJorProx() {

// If we would have a JOR CPU implementation do some decision here if we use the GPU JOR Prox velocity or the JOR CPU

#if HAVE_CUDA_SUPPORT == 1

    LOGSLLEVEL2(m_pSolverLog,  "---> using JOR Prox Velocity (GPU): "<< std::endl;);
    // TODO

#else
    ASSERTMSG(false,"InclusionSolverCONoG:: CPU JOR Prox iteration not implemented!");
    // Jor Prox iteration on CPU will not be implemented as the Full SOR and Contact SOR algorithms are much better!
#endif
}

template<bool onlyNotInContactGraph> // default to false = all bodies
void InclusionSolverCONoG::integrateAllBodyVelocities() {
    for( auto & bodyPtr : m_simBodies){
        if( (onlyNotInContactGraph && !bodyPtr->m_pSolverData->m_bInContactGraph) || onlyNotInContactGraph == false){
           bodyPtr->m_pSolverData->m_uBuffer.m_front += bodyPtr->m_pSolverData->m_uBuffer.m_back + bodyPtr->m_MassMatrixInv_diag.asDiagonal()  *  bodyPtr->m_h_term * m_settings.m_deltaT;
        }
    }
}



void InclusionSolverCONoG::initContactGraphForIteration(PREC alpha) {

    // Init Graph for iteration;
    m_contactGraph.initForIteration();


    m_pSorProxInitNodeVisitor->setParams(alpha);

    if(m_settings.m_eMethod == InclusionSolverSettings::Method::SOR_NORMAL_TANGENTIAL ){
        m_pNormalSorProxStepNodeVisitor->setParams(alpha);
        m_pTangentialSorProxStepNodeVisitor->setParams(alpha);
    }else{
        m_pSorProxStepNodeVisitor->setParams(alpha);
    }

    // Calculates b vector for all nodes, u_0, R_ii, ...
    m_contactGraph.applyNodeVisitor(*m_pSorProxInitNodeVisitor);



    // Integrate all bodies and save m_back, this integration needs to be after applyNode above (because m_back is used !!!) !
    for(auto & bodyPtr : m_simBodies) {
        // All bodies also the ones not in the contact graph...
        // add u_s + M^⁻1*h*deltaT ,  all contact forces initial values have already been applied!
        bodyPtr->m_pSolverData->m_uBuffer.m_front += bodyPtr->m_pSolverData->m_uBuffer.m_back + bodyPtr->m_MassMatrixInv_diag.asDiagonal()  *  bodyPtr->m_h_term * m_settings.m_deltaT;
        bodyPtr->m_pSolverData->m_uBuffer.m_back  = bodyPtr->m_pSolverData->m_uBuffer.m_front; // Used for cancel criteria
    }

}


void InclusionSolverCONoG::doSorProx() {

    // Do some decision according to the number of contacts if we use the GPU or the CPU

    #if HAVE_CUDA_SUPPORT == 1
        bool gpuSuccess = true;
        bool goOnGPU = m_jorProxGPUModule.computeOnGPU(m_pCollisionSolver->getCollisionSetRef().size(), m_contactGraph.m_simBodiesToContactsMap.size());

        if(goOnGPU){
           doJORProxGPU();
           //doSORProxCPU();
        }else{
           doSORProxCPU();
        }

    #else

       doSORProxCPU();

    #endif // HAVE_CUDA_SUPPORT



}

void InclusionSolverCONoG::doJORProxGPU(){
    #if HAVE_CUDA_SUPPORT == 1
        LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Using JOR Prox Velocity (GPU): "<< std::endl;);
        auto & nodeList = m_contactGraph.getNodeList();


        m_jorProxGPUModule.runOnGPU(nodeList, m_contactGraph.m_simBodiesToContactsMap, m_settings.m_alphaJORProx );

        // Integrate all bodies which have not been in the contact graph:
        integrateAllBodyVelocities<true>();

    #else
        ERRORMSG("Calling this function without CUDA Support is wrong!")
    #endif
}

void InclusionSolverCONoG::doSORProxCPU(){
    LOGSLLEVEL1_CONTACT(m_pSolverLog,  "---> Using SOR Prox Velocity (CPU): "<< std::endl;);
    initContactGraphForIteration(m_settings.m_alphaSORProx);

    LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> u_e = [ ");
    for(auto it = m_simBodies.begin(); it != m_simBodies.end(); ++it) {
        LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t uBack: " << (*it)->m_pSolverData->m_uBuffer.m_back.transpose() <<std::endl);
        LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
    }
    LOGSLLEVEL3_CONTACT(m_pSolverLog, " ]" << std::endl);


    // General stupid Prox- Iteration
    while(true) {

        // Set global flag to true, if it is not converged we set it to false
        m_bConverged = true;
        LOGSLLEVEL2_CONTACT(m_pSolverLog,"---> Next iteration: "<< m_globalIterationCounter << std::endl);
        // Do one Sor Prox Iteration
        sorProxOverAllNodes();

        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> u_e = [ ");
        for(auto it = m_simBodies.begin(); it != m_simBodies.end(); ++it) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t uFront: " <<(*it)->m_pSolverData->m_uBuffer.m_front.transpose()<<std::endl);
        }
        LOGSLLEVEL3_CONTACT(m_pSolverLog, " ]" << std::endl);

        m_globalIterationCounter++;

        if ( (m_bConverged == true || m_globalIterationCounter >= m_settings.m_MaxIter)
             && m_globalIterationCounter >= m_settings.m_MinIter) {
                LOGSLLEVEL1_CONTACT(m_pSolverLog, "---> converged = "<<m_bConverged<< "\t"<< "iterations: " <<m_globalIterationCounter <<" / "<<  m_settings.m_MaxIter<< std::endl;);
            break;
        }
        #if OUTPUT_SIMDATAITERATION_FILE == 1
            if(m_globalIterationCounter >= m_settings.m_MinIter){
                m_iterationDataFile << m_maxResidual << "\t";
            }
        #endif // OUTPUT_SIMDATAITERATION_FILE
    }


    #if OUTPUT_SIMDATAITERATION_FILE == 1
        m_iterationDataFile << std::endl;
    #endif // OUTPUT_SIMDATAITERATION_FILE

    }


void InclusionSolverCONoG::sorProxOverAllNodes() {

    m_maxResidual  = 0;


    // Move over all nodes, and do a sor prox step
    //m_contactGraph.shuffleNodesUniformly(1000);

    if(m_settings.m_eMethod == InclusionSolverSettings::Method::SOR_NORMAL_TANGENTIAL ){

        //Iterate multiple times the normal direction before going to the tangential direction!
        m_pNormalSorProxStepNodeVisitor->setLastUpdate(false);
        for(int i = 0;i<4;i++){
            m_contactGraph.applyNodeVisitorSpecial(*m_pNormalSorProxStepNodeVisitor);
        }
        m_pNormalSorProxStepNodeVisitor->setLastUpdate(true);
        m_contactGraph.applyNodeVisitorSpecial(*m_pNormalSorProxStepNodeVisitor);

        m_contactGraph.applyNodeVisitorSpecial(*m_pTangentialSorProxStepNodeVisitor);
    }else{
        m_contactGraph.applyNodeVisitorSpecial(*m_pSorProxStepNodeVisitor);
    }

    m_maxResidual = m_contactGraph.m_maxResidual;
    // Move over all nodes, end of Sor Prox

    // Apply convergence criteria (Velocity) over all bodies which are in the ContactGraph
    bool converged;
    PREC residual;

    if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocity) {

        //std::cout << "Bodies: " << m_contactGraph.m_simBodiesToContactsMap.size() << std::endl;
        for(auto it=m_contactGraph.m_simBodiesToContactsMap.begin(); it !=m_contactGraph.m_simBodiesToContactsMap.end(); ++it) {
            if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) )  {

                converged = Numerics::cancelCriteriaValue(  it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
                                                            it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
                                                            m_settings.m_AbsTol,
                                                            m_settings.m_RelTol,
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

    }else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyVelocity){

        for(auto it=m_contactGraph.m_simBodiesToContactsMap.begin(); it !=m_contactGraph.m_simBodiesToContactsMap.end(); ++it) {
            if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual)) {

                converged = Numerics::cancelCriteriaMatrixNormSq( it->first->m_pSolverData->m_uBuffer.m_back, // these are the old values (got switched)
                                                                it->first->m_pSolverData->m_uBuffer.m_front, // these are the new values (got switched)
                                                                it->first->m_MassMatrix_diag,
                                                                m_settings.m_AbsTol,
                                                                m_settings.m_RelTol,
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

    m_contactGraph.resetAfterOneIteration(m_globalIterationCounter);

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


#include "InclusionSolverCO.hpp"

#include "MatrixHelpers.hpp"
#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif

#include "CPUTimer.hpp"


const unsigned int InclusionSolverCO::NDOFFriction = ContactModels::UnilateralAndCoulombFrictionContactModel::nDOFFriction;
const unsigned int InclusionSolverCO::ContactDim = ContactModels::UnilateralAndCoulombFrictionContactModel::ConvexSet::Dimension;


InclusionSolverCO::InclusionSolverCO(std::shared_ptr< CollisionSolverType >  pCollisionSolver,
                                     std::shared_ptr<DynamicsSystemType> pDynSys):
    m_SimBodies(pDynSys->m_SimBodies),
    m_Bodies(pDynSys->m_Bodies), m_ContactGraph(&(pDynSys->m_ContactParameterMap)),
    m_pCollisionSolver(pCollisionSolver),
    m_pDynSys(pDynSys) {

    if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->addContactDelegate(
        CollisionSolverType::ContactDelegateType::from_method< ContactGraphType,  &ContactGraphType::addNode>(&m_ContactGraph)
    );


    m_nContacts = 0;
    m_nLambdas = 0;

    m_globalIterationCounter =0;
    m_bConverged = false;
    m_G_conditionNumber = 0;
    m_G_notDiagDominant = 0;

}


void InclusionSolverCO::initializeLog( Logging::Log* pSolverLog,  boost::filesystem::path folder_path ) {
    m_pSolverLog = pSolverLog;


#if HAVE_CUDA_SUPPORT == 1

#endif
}

void InclusionSolverCO::reset() {

    m_pDynSys->getSettings(m_Settings);

    m_pDynSys->initMassMatrixAndHTerm();  //TODO what does that make here?

    resetForNextIter();

#if HAVE_CUDA_SUPPORT == 1
    LOG(m_pSimulationLog, "Try to set GPU Device : "<< m_Settings.m_UseGPUDeviceId << std::endl;);

    CHECK_CUDA(cudaSetDevice(m_Settings.m_UseGPUDeviceId));
    cudaDeviceProp props;
    CHECK_CUDA(cudaGetDeviceProperties(&props,m_Settings.m_UseGPUDeviceId));

    LOG(m_pSimulationLog,  "Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << std::endl;);
#endif


}


void InclusionSolverCO::resetForNextIter() {
    m_nContacts = 0;
    m_nLambdas = 0;

    m_globalIterationCounter =0;
    m_bConverged = false;
    m_G_conditionNumber = 0;
    m_G_notDiagDominant = 0;

    m_ContactGraph.clearGraph();
}




void InclusionSolverCO::swapPercussionBuffer() {
    std::swap(m_P_back,m_P_front);
}


void InclusionSolverCO::resetPercussionBuffer() {
    m_P_back = &m_P_1;
    m_P_front = &m_P_2;
}



void InclusionSolverCO::solveInclusionProblem() {

#if CoutLevelSolver>1
    LOG(m_pSolverLog, " % -> solveInclusionProblem(): "<< std::endl;);
#endif


    // Iterate over all nodes set and assemble the matrices...
    typename ContactGraphType::NodeListType & nodes = m_ContactGraph.getNodeList();
    m_nContacts = (unsigned int)nodes.size();

    m_globalIterationCounter = 0;
    m_bConverged = false;
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;

    // Update all body velocities
    static RigidBodyType * pBody;
    for(auto it= m_SimBodies.begin(); it != m_SimBodies.end(); ++it) {
        pBody = *it;
        pBody->m_pSolverData->m_uBuffer.m_front = pBody->m_pSolverData->m_uBuffer.m_back
                                                + pBody->m_MassMatrixInv_diag.asDiagonal()*pBody->m_h_term * m_Settings.m_deltaT;
    }


    if(m_nContacts > 0) {

#if CoutLevelSolver>0
        LOG(m_pSolverLog, " % nContacts = "<< m_nContacts<< std::endl;);
#endif

        m_nLambdas = m_ContactGraph.getNLambdas();
        ASSERTMSG(m_ContactGraph.getNContactModelsUsed() == 1, "ContactGraph uses not homogen contact models!")

        if(nodes[0]->m_nodeData.m_contactParameter.m_contactModel != ContactModels::ContactModelEnum::UCF_ContactModel){
            ERRORMSG("The only supported contact model so far is: ContactModels::ContactModelEnum::UCF_ContactModel")
        }
        // Assign Space for matrices =====================================
        m_mu.resize(nodes.size());

        m_P_1.resize(m_nLambdas);
        m_P_2.resize(m_nLambdas);

        m_T.setZero(m_nLambdas,m_nLambdas);

        m_d.setZero(m_nLambdas);

        m_R.resize(m_nLambdas);
        // ==============================================================

        resetPercussionBuffer();
        P_back.setZero();

        // Assemble W_N and W_T and xi_N and xi_T =====================================================
        static const CollisionData * pCollData;

        static VectorDyn I_plus_eps(ContactDim);
        static MatrixDyn G_part(ContactDim,ContactDim);
        static const MatrixUBodyDyn * W_j_body;
        static const MatrixUBodyDyn * W_i_body;
        static MatrixDynUBody W_i_bodyT_M_body;

        for (auto & currentContactNode : nodes) {

            unsigned int i = currentContactNode->m_nodeNumber; // the current node number
            unsigned int j;                                    // the current node number of the incident node!

            pCollData = currentContactNode->m_nodeData.m_pCollData;

            // Write mu parameters to m_mu
            m_mu(i) = currentContactNode->m_nodeData.m_contactParameter.m_params[2];

            I_plus_eps(0) = 1+ currentContactNode->m_nodeData.m_contactParameter.m_params[0];
            I_plus_eps(1) = 1+ currentContactNode->m_nodeData.m_contactParameter.m_params[1];
            I_plus_eps(2) = I_plus_eps(1);

            // iterate over all edges in current contact to build up G;
            typename ContactGraphType::EdgeListIteratorType it;
            RigidBodyType * edgesBody;

            for(it = currentContactNode->m_edgeList.begin(); it != currentContactNode->m_edgeList.end(); ++it) {

                edgesBody = (*it)->m_edgeData.m_pBody;
                unsigned int bodyId = edgesBody->m_id;



                if( (*it)->m_startNode->m_nodeNumber  !=  i ) {
                    // If not an outgoing edge, continue
                    continue;
                }

                j = (*it)->m_endNode->m_nodeNumber;

                // Compute G (T is used, will be later completed to T)
                // Calculate G (Start = pNode (i) , End= (*it) (j) )
                W_i_body = ContactGraphType::getW_body(currentContactNode->m_nodeData,edgesBody);

                if(i == j) { // We are on a self referencing edge! Thats good build diagonal of G and parts of c

                    W_i_bodyT_M_body = (*W_i_body).transpose() * edgesBody->m_MassMatrixInv_diag.asDiagonal();
                    m_T.block<(ContactDim),(ContactDim)>((ContactDim)*i,(ContactDim)*j).noalias()  += W_i_bodyT_M_body * (*W_i_body);
                    m_d.segment<ContactDim>((ContactDim)*i).noalias() +=  W_i_bodyT_M_body * edgesBody->m_h_term * m_Settings.m_deltaT +
                            I_plus_eps.asDiagonal()*( (*W_i_body).transpose() * edgesBody->m_pSolverData->m_uBuffer.m_back );
                } else {
                    W_j_body = ContactGraphType::getW_body((*it)->m_endNode->m_nodeData, edgesBody);
                    G_part = (*W_i_body).transpose() * edgesBody->m_MassMatrixInv_diag.asDiagonal() * (*W_j_body);
                    m_T.block<(ContactDim),(ContactDim)>((ContactDim)*i,(ContactDim)*j).noalias() += G_part;
                    m_T.block<(ContactDim),(ContactDim)>((ContactDim)*j,(ContactDim)*i).noalias() += G_part.transpose();
                }

            }

            // add once xi to c (d is used will be later completed to d)
            m_d.segment<ContactDim>((ContactDim)*i).noalias() +=  I_plus_eps.asDiagonal() * currentContactNode->m_nodeData.m_chi;



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
        LOG(m_pSolverLog,  " G= ..."<< std::endl << m_T.format(MyIOFormat::Matlab)<<";"<<std::endl;);
#endif

#if CoutLevelSolverWhenContact>2
        LOG(m_pSolverLog, " c= " << m_d.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;)
#endif

#if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog,  " P_back= "<<P_back.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;);
#endif


        if( m_Settings.m_eMethod == InclusionSolverSettingsType::SOR_FULL) {
            // Calculate  R_N, R_T,
            setupRMatrix(m_Settings.m_alphaSORProx);
            m_T = (-m_R).asDiagonal()*m_T;
            m_T.diagonal().array() += 1 ;
            m_d.noalias() = (-m_R).asDiagonal()*m_d;

#if MEASURE_TIME_PROX == 1
            CPUTimer counter;
            counter.start();
#endif

            doSorProx();

#if MEASURE_TIME_PROX == 1
            m_timeProx = counter.elapsedSec();
#endif
        } else if(m_Settings.m_eMethod == InclusionSolverSettingsType::JOR) {
            // Calculate  R_N, R_T,
            setupRMatrix(m_Settings.m_alphaJORProx);
            m_T = (-m_R).asDiagonal()*m_T;
            m_T.diagonal().array() += 1 ;
            m_d.noalias() = (-m_R).asDiagonal()*m_d;

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

#if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog,  " P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;);
#endif


        if(m_Settings.m_bIsFiniteCheck) {
            if(!MatrixHelpers::isfinite(P_front)) {
                m_isFinite = 0;
            } else {
                m_isFinite = 1;
            }
#if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog,  " % Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);
#endif
        }


#if CoutLevelSolverWhenContact>0
        LOG(m_pSolverLog,  " % Prox Iterations needed: "<< m_globalIterationCounter <<std::endl;);
#endif

        // Add deltaVelocities from lambdas to front velocity
        static VectorUBody delta_u_E;
        RigidBodyType * pBody;


        for( auto & node : nodes){

            pBody = node->m_nodeData.m_pCollData->m_pBody1;
            if( pBody->m_eState == RigidBodyType::BodyMode::SIMULATED ) {
                pBody->m_pSolverData->m_uBuffer.m_front +=
                pBody->m_MassMatrixInv_diag.asDiagonal() * node->m_nodeData.m_W_body1 * (*m_P_front).segment<ContactDim>(ContactDim* node->m_nodeNumber);
            }

            pBody = node->m_nodeData.m_pCollData->m_pBody2;
            if( pBody->m_eState == RigidBodyType::BodyMode::SIMULATED ) {
                pBody->m_pSolverData->m_uBuffer.m_front +=
                pBody->m_MassMatrixInv_diag.asDiagonal() * node->m_nodeData.m_W_body2 * (*m_P_front).segment<ContactDim>(ContactDim* node->m_nodeNumber);
            }

        }

    }

}


void InclusionSolverCO::setupRMatrix(PREC alpha) {
    PREC r_T_i;
    for(unsigned int i=0; i<m_nContacts; i++) {
        m_R((ContactDim)*i) =  alpha / m_T((ContactDim)*i,(ContactDim)*i);
        r_T_i = alpha / (m_T.diagonal().segment<NDOFFriction>((ContactDim)*i+1)).maxCoeff();
        m_R((ContactDim)*i+1) = r_T_i;
        m_R((ContactDim)*i+2) = r_T_i;
    }


#if CoutLevelSolverWhenContact>2
    LOG(m_pSolverLog, " R= "<< "diag(" << m_R.transpose().format(MyIOFormat::Matlab)<<"');"<<std::endl;);
#endif

}


void InclusionSolverCO::doJorProx() {

#if HAVE_CUDA_SUPPORT == 1
    bool gpuSuccess = true;
    bool goOnGPU = m_nContacts >= m_jorGPUVariant.getTradeoff();

    if( m_Settings.m_bUseGPU && goOnGPU ) {
#if CoutLevelSolverWhenContact>0
        m_pSolverLog->logMessage("---> Using GPU JOR...");
#endif
        m_jorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
        gpuSuccess = m_jorGPUVariant.runGPUPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_jorGPUVariant.m_nIterGPU;
        m_proxIterationTime = m_jorGPUVariant.m_gpuIterationTime*1e-3;
        m_bUsedGPU = true;
    }

    if( !m_Settings.m_bUseGPU || !gpuSuccess || !goOnGPU) {
#if CoutLevelSolverWhenContact>0
        m_pSolverLog->logMessage("---> Using CPU JOR...");
#endif
        m_jorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
        m_jorGPUVariant.runCPUEquivalentPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_jorGPUVariant.m_nIterCPU;
        m_proxIterationTime = m_jorGPUVariant.m_cpuIterationTime*1e-3;
        m_bUsedGPU = false;
    }
    m_bConverged = (m_globalIterationCounter < m_Settings.m_MaxIter)? true : false;

#else

#if CoutLevelSolverWhenContact>0
    m_pSolverLog->logMessage(" % Using CPU JOR (general)...");
#endif

    m_bUsedGPU = false;

    // General stupid Prox- Iteration
    while(true) {

        P_front.noalias() = m_T *P_back + m_d;
        Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxMulti(m_mu,P_front);

        //Calculate CancelCriteria
        m_bConverged = Numerics::cancelCriteriaValue(P_back,P_front, m_Settings.m_AbsTol, m_Settings.m_RelTol);

#if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog, " P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;);
#endif

        m_globalIterationCounter++;

        if (m_bConverged == true || m_globalIterationCounter >= m_Settings.m_MaxIter) {

#if CoutLevelSolverWhenContact>0
            LOG(m_pSolverLog,  " converged = "<<m_bConverged<< "\t"<< "iterations:" <<m_globalIterationCounter <<"/"<<  m_Settings.m_MaxIter<< std::endl;);
#endif
            break;
            // P_front contains newest values
        }

        swapPercussionBuffer();
    }

#endif

}


void InclusionSolverCO::doSorProx() {

    static VectorDyn PContact_back(NDOFFriction);
    static unsigned int counterConverged;



#if HAVE_CUDA_SUPPORT == 1
    bool gpuSuccess = true;
    bool goOnGPU = m_nContacts >= m_sorGPUVariant.getTradeoff();

    if( m_Settings.m_bUseGPU && goOnGPU) {
        #if CoutLevelSolverWhenContact>0
                m_pSolverLog->logMessage(" % Using GPU SOR...");
        #endif
        m_sorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
        P_back.setZero();
        gpuSuccess = m_sorGPUVariant.runGPUPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_sorGPUVariant.m_nIterGPU;
        m_proxIterationTime = m_sorGPUVariant.m_gpuIterationTime*1e-3;
        m_bUsedGPU = true;
    }
    if( !m_Settings.m_bUseGPU || !gpuSuccess || !goOnGPU) {

        #if CoutLevelSolverWhenContact>0
                m_pSolverLog->logMessage(" % Using CPU SOR...");
        #endif
        m_sorGPUVariant.setSettings(m_Settings.m_MaxIter,m_Settings.m_AbsTol,m_Settings.m_RelTol);
        m_sorGPUVariant.runCPUEquivalentPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_sorGPUVariant.m_nIterCPU;
        m_proxIterationTime = m_sorGPUVariant.m_cpuIterationTime*1e-3;
        m_bUsedGPU = false;
    }
    m_bConverged = (m_globalIterationCounter < m_Settings.m_MaxIter)? true : false;
#else

#if CoutLevelSolverWhenContact>0
    m_pSolverLog->logMessage(" % Using CPU SOR (general)...");
#endif

    m_bUsedGPU = false;

    // P_back is filled with initial values, swap so P_front is filled with initial values, and only work with P_front;
    swapPercussionBuffer();

    // Prox- Iteration
    while (true) {
        m_bConverged = false;
        counterConverged = 0;

        // Prox all, with sequential Sor Style..
        for(unsigned int i=0; i < m_nContacts; i++) {

            // Prox the contact
            PContact_back.noalias() = P_front.template segment<ContactDim>((ContactDim)*i) ; // Save last values

            // This statement looks like aliasing but it does not! All Matrices*Matrices or MAtrices*Vector get evaluated into temporary by default
            // TODO Check if there is a Aliasing effect
            P_front.template segment<ContactDim>((ContactDim)*i) = m_T.block((ContactDim)*i,0,ContactDim,m_nLambdas) * P_front + m_d.template segment<ContactDim>((ContactDim)*i);

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(m_mu(i),P_front.template segment<ContactDim>((ContactDim)*i));
            Numerics::cancelCriteriaValue(PContact_back, P_front.template segment<ContactDim>((ContactDim)*i), m_Settings.m_AbsTol, m_Settings.m_RelTol, counterConverged);

        }

#if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog, " P_front= "<<P_front.transpose().format(MyIOFormat::Matlab)<<"';"<<std::endl;);
#endif

        m_globalIterationCounter++;

        if (counterConverged == m_nContacts || m_globalIterationCounter >= m_Settings.m_MaxIter) {
            m_bConverged = true;
            // P_front has newest values.
            break;
        }
    }

#endif
}



std::string InclusionSolverCO::getIterationStats() {
    std::stringstream s;

    s   << m_bUsedGPU<<"\t"
        << m_nContacts<<"\t"
        << m_globalIterationCounter<<"\t"
        << m_bConverged<<"\t"
        << m_isFinite<<"\t"
        << m_timeProx<<"\t"
        << m_proxIterationTime<<"\t"
        << m_pDynSys->m_currentTotEnergy <<"\t"
        << m_G_conditionNumber<<"\t" //No m_G_conditionNumber
        << m_G_notDiagDominant; //No m_G_notDiagDominant
    return s.str();
}

std::string InclusionSolverCO::getStatsHeader() {
    std::stringstream s;
    s
    << "GPUUsed"<<"\t"
    << "nContacts"<<"\t"
    << "nGlobalIterations"<<"\t"
    << "Converged"<<"\t"
    << "IsFinite"<<"\t"
    << "TotalTimeProx [s]"<<"\t"
    << "IterTimeProx [s]"<<"\t"
    << "TotalStateEnergy [J]" <<"\t"
    << "ConditionNumberG"<<"\t" //No m_G_conditionNumber
    << "GnotDiagDom"; //No m_G_notDiagDominant
    return s.str();
}


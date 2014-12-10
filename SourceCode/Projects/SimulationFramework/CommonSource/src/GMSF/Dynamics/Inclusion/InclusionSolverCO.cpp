
#include "GMSF/Dynamics/Inclusion/InclusionSolverCO.hpp"

#include "GMSF/Dynamics/General/MatrixHelpers.hpp"
#include "GMSF/Dynamics/General/VectorToSkewMatrix.hpp"
#include "GMSF/Dynamics/Inclusion/ProxFunctions.hpp"

#if HAVE_CUDA_SUPPORT == 1
#include "JorProxGPUVariant.hpp"
#include "SorProxGPUVariant.hpp"
#endif

#include "GMSF/Common/CPUTimer.hpp"


const unsigned int InclusionSolverCO::NDOFFriction = CONTACTMODELTYPE(ContactModels::Enum::UCF)::nDOFFriction;
const unsigned int InclusionSolverCO::ContactDim = CONTACTMODELTYPE(ContactModels::Enum::UCF)::ConvexSet::Dimension;


InclusionSolverCO::InclusionSolverCO(std::shared_ptr< CollisionSolverType >  pCollisionSolver,
                                     std::shared_ptr<DynamicsSystemType> pDynSys):
    m_simBodies(pDynSys->m_simBodies),
    m_staticBodies(pDynSys->m_staticBodies), m_contactGraph(&(pDynSys->m_ContactParameterMap)),
    m_pCollisionSolver(pCollisionSolver),
    m_pDynSys(pDynSys) {

    if(Logging::LogManager::getSingleton().existsLog("SimulationLog")) {
        m_pSimulationLog = Logging::LogManager::getSingleton().getLog("SimulationLog");
    } else {
        ERRORMSG("There is no SimulationLog in the LogManager... Did you create it?")
    }

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

    m_settings = m_pDynSys->getSettingsInclusionSolver();


    resetForNextTimestep();

    //Add a delegate function in the Contact Graph, which add the new Contact given by the CollisionSolver
    m_pCollisionSolver->addContactDelegate(
        CollisionSolverType::ContactDelegateType::from_method< ContactGraphType,  &ContactGraphType::addNode>(&m_contactGraph)
    );


#if HAVE_CUDA_SUPPORT == 1
    LOG(m_pSimulationLog, "Try to set GPU Device : "<< m_settings.m_UseGPUDeviceId << std::endl;);

    CHECK_CUDA(cudaSetDevice(m_settings.m_UseGPUDeviceId));
    cudaDeviceProp props;
    CHECK_CUDA(cudaGetDeviceProperties(&props,m_settings.m_UseGPUDeviceId));

    LOG(m_pSimulationLog,  "Set GPU Device : "<< props.name << ", PCI Bus Id: "<<props.pciBusID << ", PCI Device Id: " << props.pciDeviceID << std::endl;);
#endif


}


void InclusionSolverCO::resetForNextTimestep() {
    m_nContacts = 0;
    m_nLambdas = 0;

    m_globalIterationCounter =0;
    m_bConverged = false;
    m_G_conditionNumber = 0;
    m_G_notDiagDominant = 0;

    m_contactGraph.clearGraph();
}




void InclusionSolverCO::swapPercussionBuffer() {
    std::swap(m_P_back,m_P_front);
}


void InclusionSolverCO::resetPercussionBuffer() {
    m_P_back = &m_P_1;
    m_P_front = &m_P_2;
}



void InclusionSolverCO::solveInclusionProblem() {


    LOGSLLEVEL2(m_pSolverLog, " % -> solveInclusionProblem(): "<< std::endl;);



    // Iterate over all nodes set and assemble the matrices...
    typename ContactGraphType::NodeListType & nodes = m_contactGraph.getNodeList();
    m_nContacts = (unsigned int)nodes.size();

    m_globalIterationCounter = 0;
    m_bConverged = false;
    m_isFinite = -1;
    m_bUsedGPU = false;
    m_timeProx = 0;
    m_proxIterationTime = 0;

    // Update all body velocities
    static RigidBodyType * pBody;
    for(auto it= m_simBodies.begin(); it != m_simBodies.end(); ++it) {
        pBody = *it;
        pBody->m_pSolverData->m_uBuffer.m_front = pBody->m_pSolverData->m_uBuffer.m_back
                                                + pBody->m_MassMatrixInv_diag.asDiagonal()*pBody->m_h_term * m_settings.m_deltaT;
    }


    if(m_nContacts > 0) {


        LOGSLLEVEL1(m_pSolverLog, " % nContacts = "<< m_nContacts<< std::endl;);


        m_nLambdas = m_contactGraph.getNLambdas();
        ASSERTMSG(m_contactGraph.getNContactModelsUsed() == 1, "ContactGraph uses not homogen contact models!")

        if(nodes[0]->m_nodeData.m_contactParameter.m_contactModel != ContactModels::Enum::UCF){
            ERRORMSG("The only supported contact model so far is: ContactModels::Enum::UCF")
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
        static MatrixDynDyn G_part(ContactDim,ContactDim);
        static const MatrixUBodyDyn * W_j_body;
        static const MatrixUBodyDyn * W_i_body;
        static MatrixDynUBody W_i_bodyT_M_body;

        for (auto & currentContactNode : nodes) {

            unsigned int i = currentContactNode->m_nodeNumber; // the current node number
            unsigned int j;                                    // the current node number of the incident node!

            pCollData = currentContactNode->m_nodeData.m_pCollData;

            // Write mu parameters to m_mu
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            m_mu(i) = currentContactNode->m_nodeData.m_contactParameter.m_params[CMT::muIdx];

            I_plus_eps(0) = 1+ currentContactNode->m_nodeData.m_contactParameter.m_params[CMT::epsNIdx];
            I_plus_eps(1) = 1+ currentContactNode->m_nodeData.m_contactParameter.m_params[CMT::epsTIdx];
            I_plus_eps(2) = I_plus_eps(1);

            // iterate over all edges in current contact to build up G;
            typename ContactGraphType::EdgeListIteratorType it;
            RigidBodyType * edgesBody;

            for(it = currentContactNode->m_edgeList.begin(); it != currentContactNode->m_edgeList.end(); ++it) {

                edgesBody = (*it)->m_edgeData.m_pBody;



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
                    m_d.segment<ContactDim>((ContactDim)*i).noalias() +=  W_i_bodyT_M_body * edgesBody->m_h_term * m_settings.m_deltaT +
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



        LOGSLLEVEL3_CONTACT(m_pSolverLog,  " G= ..."<< std::endl << m_T.format(MyMatrixIOFormat::Matlab)<<";"<<std::endl;);
        LOGSLLEVEL3_CONTACT(m_pSolverLog, " c= " << m_d.transpose().format(MyMatrixIOFormat::Matlab)<<"';"<<std::endl;)
        LOGSLLEVEL2_CONTACT(m_pSolverLog,  " P_back= "<<P_back.transpose().format(MyMatrixIOFormat::Matlab)<<"';"<<std::endl;);



        if( m_settings.m_eMethod == InclusionSolverSettingsType::SOR_FULL) {
            // Calculate  R_N, R_T,
            setupRMatrix(m_settings.m_alphaSORProx);
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
        } else if(m_settings.m_eMethod == InclusionSolverSettingsType::JOR) {
            // Calculate  R_N, R_T,
            setupRMatrix(m_settings.m_alphaJORProx);
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


        LOGSLLEVEL2_CONTACT(m_pSolverLog,  " P_front= "<<P_front.transpose().format(MyMatrixIOFormat::Matlab)<<"';"<<std::endl;);



        if(m_settings.m_bIsFiniteCheck) {
            if(!MatrixHelpers::isfinite(P_front)) {
                m_isFinite = 0;
            } else {
                m_isFinite = 1;
            }

            LOGSLLEVEL1_CONTACT(m_pSolverLog,  " % Solution of Prox Iteration is finite: "<< m_isFinite <<std::endl;);

        }



        LOGSLLEVEL1_CONTACT(m_pSolverLog,  " % Prox Iterations needed: "<< m_globalIterationCounter <<std::endl;);


        // Add deltaVelocities from lambdas to front velocity
        static VectorUBody delta_u_E;
        RigidBodyType * pBody;


        for( auto & node : nodes){

            pBody = node->m_nodeData.m_pCollData->m_pBody1;
            if( pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                pBody->m_pSolverData->m_uBuffer.m_front +=
                pBody->m_MassMatrixInv_diag.asDiagonal() * node->m_nodeData.m_W_body1 * (*m_P_front).segment<ContactDim>(ContactDim* node->m_nodeNumber);
            }

            pBody = node->m_nodeData.m_pCollData->m_pBody2;
            if( pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
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



    LOGSLLEVEL3_CONTACT(m_pSolverLog, " R= "<< "diag(" << m_R.transpose().format(MyMatrixIOFormat::Matlab)<<"');"<<std::endl;);


}


void InclusionSolverCO::doJorProx() {

#if HAVE_CUDA_SUPPORT == 1
    bool gpuSuccess = true;
    bool goOnGPU = m_nContacts >= m_jorGPUVariant.getTradeoff();

    if( m_settings.m_bUseGPU && goOnGPU ) {

        LOGSLLEVEL1_CONTACT(m_pSolverLog,"---> Using GPU JOR...");

        m_jorGPUVariant.setSettings(m_settings.m_MaxIter,m_settings.m_AbsTol,m_settings.m_RelTol);
        gpuSuccess = m_jorGPUVariant.runGPUPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_jorGPUVariant.m_nIterGPU;
        m_proxIterationTime = m_jorGPUVariant.m_gpuIterationTime*1e-3;
        m_bUsedGPU = true;
    }

    if( !m_settings.m_bUseGPU || !gpuSuccess || !goOnGPU) {

        LOGSLLEVEL1_CONTACT(m_pSolverLog,"---> Using CPU JOR...");

        m_jorGPUVariant.setSettings(m_settings.m_MaxIter,m_settings.m_AbsTol,m_settings.m_RelTol);
        m_jorGPUVariant.runCPUEquivalentPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_jorGPUVariant.m_nIterCPU;
        m_proxIterationTime = m_jorGPUVariant.m_cpuIterationTime*1e-3;
        m_bUsedGPU = false;
    }
    m_bConverged = (m_globalIterationCounter < m_settings.m_MaxIter)? true : false;

#else


    LOGSLLEVEL1_CONTACT(m_pSolverLog, " % Using CPU JOR (general)...");


    m_bUsedGPU = false;

    // General stupid Prox- Iteration
    while(true) {

        P_front.noalias() = m_T *P_back + m_d;
        Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxMulti(m_mu,P_front);

        //Calculate CancelCriteria
        m_bConverged = Numerics::cancelCriteriaValue(P_back,P_front, m_settings.m_AbsTol, m_settings.m_RelTol);


        LOGSLLEVEL2_CONTACT(m_pSolverLog, " P_front= "<<P_front.transpose().format(MyMatrixIOFormat::Matlab)<<"';"<<std::endl;);


        m_globalIterationCounter++;

        if (m_bConverged == true || m_globalIterationCounter >= m_settings.m_MaxIter) {


            LOGSLLEVEL1_CONTACT(m_pSolverLog,  " converged = "<<m_bConverged<< "\t"<< "iterations:" <<m_globalIterationCounter <<"/"<<  m_settings.m_MaxIter<< std::endl;);

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

    if( m_settings.m_bUseGPU && goOnGPU) {

        LOGSLLEVEL1_CONTACT(m_pSolverLog," % Using GPU SOR...");

        m_sorGPUVariant.setSettings(m_settings.m_MaxIter,m_settings.m_AbsTol,m_settings.m_RelTol);
        P_back.setZero();
        gpuSuccess = m_sorGPUVariant.runGPUPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_sorGPUVariant.m_nIterGPU;
        m_proxIterationTime = m_sorGPUVariant.m_gpuIterationTime*1e-3;
        m_bUsedGPU = true;
    }
    if( !m_settings.m_bUseGPU || !gpuSuccess || !goOnGPU) {


        LOGSLLEVEL1_CONTACT(m_pSolverLog," % Using CPU SOR...");

        m_sorGPUVariant.setSettings(m_settings.m_MaxIter,m_settings.m_AbsTol,m_settings.m_RelTol);
        m_sorGPUVariant.runCPUEquivalentPlain(P_front,m_T,P_back,m_d,m_mu);
        m_globalIterationCounter = m_sorGPUVariant.m_nIterCPU;
        m_proxIterationTime = m_sorGPUVariant.m_cpuIterationTime*1e-3;
        m_bUsedGPU = false;
    }
    m_bConverged = (m_globalIterationCounter < m_settings.m_MaxIter)? true : false;
#else


    LOGSLLEVEL1_CONTACT(m_pSolverLog," % Using CPU SOR (general)...");


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
            Numerics::cancelCriteriaValue(PContact_back, P_front.template segment<ContactDim>((ContactDim)*i), m_settings.m_AbsTol, m_settings.m_RelTol, counterConverged);

        }


        LOGSLLEVEL2_CONTACT(m_pSolverLog, " P_front= "<<P_front.transpose().format(MyMatrixIOFormat::Matlab)<<"';"<<std::endl;);


        m_globalIterationCounter++;

        if (counterConverged == m_nContacts || m_globalIterationCounter >= m_settings.m_MaxIter) {
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


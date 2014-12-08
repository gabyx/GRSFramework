
#include "GMSF/Dynamics/Inclusion/ContactGraphMPI.hpp"


#include "GMSF/Dynamics/Inclusion/ContactFeasibilityTable.hpp"
template<typename Combo>
ContactGraph<Combo>::ContactGraph(std::shared_ptr<DynamicsSystemType> pDynSys):
    m_nodeCounter(0),
    m_edgeCounter(0),
    m_nLambdas(0),
    m_nFrictionParams(0),
    m_pDynSys(pDynSys),
    m_pContactParameterMap(&pDynSys->m_ContactParameterMap)
{}
template<typename Combo>
void ContactGraph<Combo>::setInclusionCommunicator(std::shared_ptr<InclusionCommunicatorType> pInclusionComm) {
    m_pInclusionComm = pInclusionComm;
    m_pNbDataMap = m_pInclusionComm->getNeighbourMap();
}
template<typename Combo>
ContactGraph::~ContactGraph() {
    clearGraph();
}
template<typename Combo>
void ContactGraph<Combo>::setLog(Logging::Log * solverLog) {
    m_pSolverLog = solverLog;
}
template<typename Combo>
void ContactGraph<Combo>::clearGraph() {
    // This deletes all nodes, edges
    // cleanup allocated memory
    for(auto n_it = this->m_nodes.begin(); n_it != this->m_nodes.end(); n_it++)
        delete (*n_it);
    for(auto e_it = this->m_edges.begin(); e_it != this->m_edges.end(); e_it++)
        delete (*e_it);
    //cout << "clear graph"<<endl;
    this->m_nodes.clear();
    m_nodeCounter = 0;
    this->m_edges.clear();
    m_edgeCounter = 0;
    m_nLambdas =0;
    m_nFrictionParams=0;
    m_SimBodyToContactsList.clear();

    //clear own lists
    m_localNodes.clear();
    m_remoteNodes.clear();

    for(auto n_it = m_splittedNodes.begin(); n_it != m_splittedNodes.end(); n_it++) {
        delete (n_it->second);
    }

    m_splittedNodes.clear();
}
template<typename Combo>
void ContactGraph<Combo>::addNode(CollisionData * pCollData) {

    //Take care state, is only q = q_m, u is not set and is zero!

    ASSERTMSG(pCollData->m_pBody1 != NULL && pCollData->m_pBody2 != NULL, " Bodys are null pointers?");
    //cout << "add node : "<<m_nodeCounter<< " body id:" << RigidBodyId::getBodyIdString(pCollData->m_pBody1) <<" and "<< RigidBodyId::getBodyIdString(pCollData->m_pBody2) <<endl;

    //  add a contact node to the graph
    // check to which nodes we need to connect?
    // all nodes (contacts) which are in the BodyContactList (maps bodies -> contacts)

    // add the pNodeData to the node list
    this->m_nodes.push_back( new NodeType(m_nodeCounter));
    NodeType * addedNode = this->m_nodes.back();
    // Add to Remote or Local list
    // Check that contact is Local-Local or Remote-Local or Local-Remote
    // So Fail if Remote-Remote
    std::pair<bool,bool> isRemote;
    bool feasible = ContactFeasibilityTableMPI::checkFeasibilityOfContact(pCollData->m_pBody1, pCollData->m_pBody2 , isRemote );
    LOGASSERTMSG(feasible, m_pSolverLog, "Contact not feasible between body:  id1: " <<pCollData->m_pBody1->m_id << " and id2: " <<pCollData->m_pBody2->m_id )

    if(isRemote.first or isRemote.second) {
        //set the node color
#if CoutLevelSolverWhenContact>1
        LOG(m_pSolverLog,"---> Added Remote Node on bodies:" << std::endl;)
#endif

        addedNode->m_nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::REMOTENODE);
        m_remoteNodes.push_back(addedNode);

        //Add to the neighbour data
        if(isRemote.first) {
#if CoutLevelSolverWhenContact>1
            LOG(m_pSolverLog,"\t---> Remote body id: "<< RigidBodyId::getBodyIdString(pCollData->m_pBody1->m_id) << std::endl;)
#endif
            m_pNbDataMap->getNeighbourData(pCollData->m_pBody1->m_pBodyInfo->m_ownerRank)->addRemoteBodyData(pCollData->m_pBody1);
            // if this body is already added it does not matter
        }
        if(isRemote.second) {
#if CoutLevelSolverWhenContact>1
            LOG(m_pSolverLog,"\t---> Remote body id: "<< RigidBodyId::getBodyIdString(pCollData->m_pBody2->m_id) << std::endl;)
#endif
            m_pNbDataMap->getNeighbourData(pCollData->m_pBody2->m_pBodyInfo->m_ownerRank)->addRemoteBodyData(pCollData->m_pBody2);
            // if this body is already added it does not matter
        }

    } else {
        //set the node color
        addedNode->m_nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::LOCALNODE);
        m_localNodes.push_back(addedNode);
    }

    addedNode->m_nodeData.m_pCollData = pCollData;



    // Specify the contact model, (here we should do a look up or what ever)! ==================================
    // TODO
    addedNode->m_nodeData.m_eContactModel = ContactModels::NCF_ContactModel;
    const unsigned int dimSet = ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension;

    addedNode->m_nodeData.m_chi.setZero(dimSet); //TODO take care, relative velocity dimesion independet of dimSet, could be?
    addedNode->m_nodeData.m_b.setZero(dimSet);
    addedNode->m_nodeData.m_LambdaBack.setZero(dimSet);
    addedNode->m_nodeData.m_LambdaFront.setZero(dimSet);
    addedNode->m_nodeData.m_R_i_inv_diag.setZero(dimSet);
    addedNode->m_nodeData.m_G_ii.setZero(dimSet,dimSet);
    addedNode->m_nodeData.m_nLambdas = dimSet;
    // =========================================================================================================

    // Compute general parameters for the contact
    computeParams(addedNode->m_nodeData);

    // FIRST BODY!
    if( pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {

        //Set Flag that this Body in ContactGraph
        pCollData->m_pBody1->m_pSolverData->m_bInContactGraph = true;
        // Unset the flag when this Node is removed;

        //Link to FrontBackBuffer
        addedNode->m_nodeData.m_u1BufferPtr = & pCollData->m_pBody1->m_pSolverData->m_uBuffer;

        computeW<1>( addedNode->m_nodeData);
        connectNode<1>( addedNode);

    } else if( pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::ANIMATED ) {
        // Contact goes into xi_N, xi_T
        ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
    }


    // SECOND BODY!
    if( pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {

        //Set Flag that this Body in ContactGraph
        pCollData->m_pBody2->m_pSolverData->m_bInContactGraph = true;
        // Unset the flag when this Node is removed;

        //Link to FrontBackBuffer
        addedNode->m_nodeData.m_u2BufferPtr = & pCollData->m_pBody2->m_pSolverData->m_uBuffer;

        computeW<2>( addedNode->m_nodeData);
        connectNode<2>( addedNode);

    } else if( pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::ANIMATED ) {
        // Contact goes into xi_N, xi_T
        ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
    }

    // increment the lambda counter, of how many forces we have so far!
    m_nLambdas += addedNode->m_nodeData.m_nLambdas;
    m_nFrictionParams += addedNode->m_nodeData.m_mu.rows();

    m_nodeCounter++;
}

template<typename Combo>
std::pair<typename ContactGraph<Combo>::SplitBodyNodeDataType *, bool> ContactGraph<Combo>::addSplitBodyNode(RigidBodyType * body, const RankIdType & rank) {
    auto pairRes = m_splittedNodes.insert(
                       SplitBodyNodeDataListType::value_type(body->m_id, (SplitBodyNodeDataType*) NULL )
                   );

    if(pairRes.second) {
        //if insered set the new pointer
        pairRes.first->second = new SplitBodyNodeDataType(body);
    }

    LOGASSERTMSG(pairRes.first->second,m_pSolverLog, "SplitBodyNode pointer of body " << body->m_id << " is zero!")
    //Set rank
    bool added = pairRes.first->second->addRank(rank);
    LOGASSERTMSG(added,m_pSolverLog, "Rank could not been added to SplitBodyNode with id: " << body->m_id)

    return std::pair<SplitBodyNodeDataType *, bool>(pairRes.first->second, pairRes.second);
}
template<typename Combo>
void ContactGraph<Combo>::computeParams(NodeDataType & nodeData) {
    if( nodeData.m_eContactModel == ContactModels::NCF_ContactModel ) {
        // Get Contact Parameters


        //Set matrix size!
        nodeData.m_I_plus_eps.setZero(ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);
        nodeData.m_eps.setZero(ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);
        nodeData.m_mu.setZero(ContactModels::NormalAndCoulombFrictionContactModel::nFrictionParams);



        ContactParams & params  = m_pContactParameterMap->getContactParams(nodeData.m_pCollData->m_pBody1->m_eMaterial,nodeData.m_pCollData->m_pBody2->m_eMaterial);
        nodeData.m_mu(0)     = params.m_mu;
        nodeData.m_eps(0)    = params.m_epsilon_N;
        nodeData.m_eps(1)    = params.m_epsilon_T;
        nodeData.m_eps(2)    = params.m_epsilon_T;

        nodeData.m_I_plus_eps.array() = nodeData.m_eps.array() + 1;



    } else {
        ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
    }

}

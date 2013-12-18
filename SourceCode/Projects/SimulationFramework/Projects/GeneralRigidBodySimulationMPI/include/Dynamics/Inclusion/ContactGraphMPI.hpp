#ifndef ContactGraphMPI_hpp
#define ContactGraphMPI_hpp


/** Contact Graph */
#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "GeneralGraph.hpp"

#include "CollisionData.hpp"

#include "ContactModels.hpp"
#include "ContactParameterMap.hpp"

#include "ProxFunctions.hpp"
#include "InclusionSolverSettings.hpp"

#include "ContactFeasibilityTable.hpp"

#include "VectorToSkewMatrix.hpp"

#include "ContactGraphNodeDataMPI.hpp"

class InclusionCommunicator;

class ContactGraph : public Graph::GeneralGraph< ContactGraphNodeDataIteration,ContactGraphEdgeData > {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    typedef typename RigidBodyType::RigidBodyIdType RigidBodyIdType;

    typedef InclusionCommunicator InclusionCommunicatorType;
    typedef typename InclusionCommunicatorType::NeighbourMapType NeighbourMapType;

    typedef ContactGraphNodeDataIteration NodeDataType;
    typedef ContactGraphEdgeData EdgeDataType;
    typedef typename Graph::Node< NodeDataType, EdgeDataType> NodeType;
    typedef typename Graph::Edge< NodeDataType, EdgeDataType> EdgeType;

    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType NodeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType EdgeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType NodeListIteratorType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType EdgeListIteratorType;

    typedef ContactGraphNodeDataSplitBody SplitBodyNodeType;
    typedef std::unordered_map<RigidBodyIdType, SplitBodyNodeType* > SplitBodyNodeListType;

    enum class NodeColor: unsigned short {LOCALNODE, REMOTENODE, SPLITNODE};

    ContactGraph(boost::shared_ptr<DynamicsSystemType> pDynSys):
        m_nodeCounter(0),
        m_edgeCounter(0),
        m_nLambdas(0),
        m_nFrictionParams(0),
        m_pDynSys(pDynSys),
        m_pContactParameterMap(&pDynSys->m_ContactParameterMap)
    {}

    void setInclusionCommunicator(boost::shared_ptr<InclusionCommunicatorType> pInclusionComm){
        m_pInclusionComm = pInclusionComm;
        m_pNbDataMap = m_pInclusionComm->getNeighbourMap();
    }

    ~ContactGraph() {
        clearGraph();
    }

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void clearGraph() {
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

        for(auto n_it = m_splittedNodes.begin(); n_it != m_splittedNodes.end(); n_it++){
            delete (n_it->second);
        }

        m_splittedNodes.clear();
    }

    void addNode(CollisionData * pCollData) {

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

        if(isRemote.first or isRemote.second){
            //set the node color
        #if CoutLevelSolverWhenContact>1
            LOG(m_pSolverLog,"---> Added Remote Node on bodies:" << std::endl;)
        #endif

            addedNode->m_nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::REMOTENODE);
            m_remoteNodes.push_back(addedNode);

            //Add to the neighbour data
            if(isRemote.first){
                #if CoutLevelSolverWhenContact>1
                    LOG(m_pSolverLog,"\t---> Remote body id: "<< RigidBodyId::getBodyIdString(pCollData->m_pBody1->m_id) << std::endl;)
                #endif
                m_pNbDataMap->getNeighbourData(pCollData->m_pBody1->m_pBodyInfo->m_ownerRank)->addRemoteBodyData(pCollData->m_pBody1);
                // if this body is already added it does not matter
            }
            if(isRemote.second){
                #if CoutLevelSolverWhenContact>1
                    LOG(m_pSolverLog,"\t---> Remote body id: "<< RigidBodyId::getBodyIdString(pCollData->m_pBody2->m_id) << std::endl;)
                #endif
                m_pNbDataMap->getNeighbourData(pCollData->m_pBody2->m_pBodyInfo->m_ownerRank)->addRemoteBodyData(pCollData->m_pBody2);
                // if this body is already added it does not matter
            }

        }else{
            //set the node color
            addedNode->m_nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::LOCALNODE);
            m_localNodes.push_back(addedNode);
        }

        addedNode->m_nodeData.m_pCollData = pCollData;



        // Specify the contact model, (here we should do a look up or what ever)! ==================================
        // TODO
        addedNode->m_nodeData.m_eContactModel = ContactModels::NCF_ContactModel;
        const unsigned int dimSet = ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension;

        addedNode->m_nodeData.m_xi.setZero(dimSet); //TODO take care, relative velocity dimesion independet of dimSet, could be?
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


    inline  const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    inline  const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    //Local Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorLocal(TNodeVisitor & vv){
		for(auto curr_node = m_localNodes.begin(); curr_node != m_localNodes.end(); curr_node++)
			(*(*curr_node)).template acceptVisitor<TNodeVisitor>(vv);
	}
	//Remote Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorRemote(TNodeVisitor & vv){
		for(auto curr_node = m_remoteNodes.begin(); curr_node != m_remoteNodes.end(); curr_node++)
			(*(*curr_node)).template acceptVisitor<TNodeVisitor>(vv);
	}



    std::unordered_map<const RigidBodyType *, NodeListType > m_SimBodyToContactsList;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.


    std::pair<SplitBodyNodeType *, bool> addSplitBodyNode(RigidBodyType * body, const RankIdType & rank){
        auto pairRes = m_splittedNodes.insert(
                                              SplitBodyNodeListType::value_type(body->m_id, (SplitBodyNodeType*) NULL )
                                                );

        if(pairRes.second){
            //if insered set the new pointer
            pairRes.first->second = new SplitBodyNodeType(body);
        }

        LOGASSERTMSG(pairRes.first->second,m_pSolverLog, "SplitBodyNode pointer of body " << body->m_id << " is zero!")
        //Set rank
        bool added = pairRes.first->second->addRank(rank);
        LOGASSERTMSG(added,m_pSolverLog, "Rank could not been added to SplitBodyNode with id: " << body->m_id)

        return std::pair<SplitBodyNodeType *, bool>(pairRes.first->second, pairRes.second);
    }

private:

    Logging::Log * m_pSolverLog;

    boost::shared_ptr<InclusionCommunicatorType> m_pInclusionComm;
    NeighbourMapType * m_pNbDataMap; ///< NeighbourMap to insert remote bodies which have contacts

    boost::shared_ptr<DynamicsSystemType> m_pDynSys;

    //std::map<unsigned int, NodeListType> m_nodeMap; //TODO make a map whith each color!
    NodeListType m_remoteNodes; ///< These are the contact nodes which lie on the remote bodies (ref to m_nodeMap)
    NodeListType m_localNodes;  ///< These are the contact nodes which lie on the local bodies (ref to m_nodeMap)

    SplitBodyNodeListType m_splittedNodes; ///< These are the billateral nodes between the splitted bodies in the contact graph



    void computeParams(NodeDataType & nodeData) {
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

    template<int bodyNr>
    inline void computeW(NodeDataType & nodeData) {



        if(nodeData.m_eContactModel == ContactModels::NCF_ContactModel) {


            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData * pCollData;

            pCollData = nodeData.m_pCollData;



            if(bodyNr == 1) {
                //Set matrix size!
                nodeData.m_W_body1.setZero(NDOFuObj, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

                updateSkewSymmetricMatrix<>( pCollData->m_r_S1C1, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody1->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body1.col(0).template head<3>() = - pCollData->m_cFrame.m_e_z; // I frame
                nodeData.m_W_body1.col(0).template tail<3>() = - I_Jacobi_2 * pCollData->m_cFrame.m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body1.col(1).template head<3>() = - pCollData->m_cFrame.m_e_x; // I frame
                nodeData.m_W_body1.col(1).template tail<3>() = - I_Jacobi_2 * pCollData->m_cFrame.m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body1.col(2).template head<3>() = - pCollData->m_cFrame.m_e_y; // I frame
                nodeData.m_W_body1.col(2).template tail<3>() = - I_Jacobi_2 * pCollData->m_cFrame.m_e_y;
            } else {
                //Set matrix size!
                nodeData.m_W_body2.setZero(NDOFuObj, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

                updateSkewSymmetricMatrix<>( pCollData->m_r_S2C2, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody2->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body2.col(0).template head<3>() =  pCollData->m_cFrame.m_e_z; // I frame
                nodeData.m_W_body2.col(0).template tail<3>() =  I_Jacobi_2 * pCollData->m_cFrame.m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body2.col(1).template head<3>() =  pCollData->m_cFrame.m_e_x; // I frame
                nodeData.m_W_body2.col(1).template tail<3>() =  I_Jacobi_2 * pCollData->m_cFrame.m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body2.col(2).template head<3>() =  pCollData->m_cFrame.m_e_y; // I frame
                nodeData.m_W_body2.col(2).template tail<3>() =  I_Jacobi_2 * pCollData->m_cFrame.m_e_y;
            }
        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }

    }

    template<int bodyNr>
    void connectNode(NodeType * pNode) {

        EdgeType * addedEdge;
        RigidBodyType * pBody = (bodyNr==1)? pNode->m_nodeData.m_pCollData->m_pBody1 : pNode->m_nodeData.m_pCollData->m_pBody2;

        // Add self edge! ===========================================================
        this->m_edges.push_back(new EdgeType(m_edgeCounter));
        addedEdge = this->m_edges.back();
        addedEdge->m_edgeData.m_pBody = pBody;

        // add links
        addedEdge->m_startNode = pNode;
        addedEdge->m_endNode = pNode;
        addedEdge->m_twinEdge = this->m_edges.back(); // Current we dont need a twin edge, self referencing!
        // Add the edge to the nodes edge list!
        pNode->m_edgeList.push_back( addedEdge );
        m_edgeCounter++;
        //cout << "add self edge: "<<pNode->m_nodeNumber<<" to "<<pNode->m_nodeNumber<<" body Id:"<< RigidBodyId::getBodyIdString(pBody)<<endl;
        // ===========================================================================

        // Get all contacts on this body and connect to them =========================
        NodeListType & nodeList = m_SimBodyToContactsList[pBody];
        //iterate over the nodeList and add edges!
        typename NodeListType::iterator it;
        // if no contacts are already on the body we skip this
        for(it = nodeList.begin(); it != nodeList.end(); it++) {

            this->m_edges.push_back(new EdgeType(m_edgeCounter));
            addedEdge = this->m_edges.back();
            addedEdge->m_edgeData.m_pBody = pBody;
            // add link
            addedEdge->m_startNode = pNode;
            addedEdge->m_endNode = (*it);
            addedEdge->m_twinEdge = addedEdge; // Current we dont need a twin edge, self referencing!
            // Add the edge to the nodes edge list!
            pNode->m_edgeList.push_back( addedEdge );
            (*it)->m_edgeList.push_back( addedEdge );
            m_edgeCounter++;
            //cout << "add edge: "<<pNode->m_nodeNumber<<" to "<<(*it)->m_nodeNumber<<" body Id:"<< RigidBodyId::getBodyIdString(pBody)<<endl;
        }

        // Add new Node to the list;
        nodeList.push_back(pNode);

    }

    ContactParameterMap* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter; ///< An edge counter, starting at 0.

};


/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxStepNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef TContactGraph ContactGraphType;
    typedef typename ContactGraphType::NodeDataType NodeDataType;
    typedef typename ContactGraphType::EdgeDataType EdgeDataType;
    typedef typename ContactGraphType::EdgeType EdgeType;
    typedef typename ContactGraphType::NodeType NodeType;

    SorProxStepNodeVisitor(const InclusionSolverSettings &settings,
                           bool & globalConverged, const unsigned int & globalIterationNeeded):
            m_Settings(settings),m_bConverged(globalConverged),
            m_iterationsNeeded(globalIterationNeeded)
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void visitNode(NodeType& node){
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorDyn uCache1,uCache2;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        #endif

        if( nodeData.m_eContactModel == ContactModels::NCF_ContactModel ) {


            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_front;
            }

#if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog,"chi: " << nodeData.m_LambdaFront.transpose() << std::endl);
#endif

            nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
            nodeData.m_LambdaFront += nodeData.m_LambdaBack;
            //Prox

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(
                nodeData.m_mu(0),
                nodeData.m_LambdaFront.template head<ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension>()
            );

#if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "Lambda Back: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOG(m_pSolverLog, "Lambda Front: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol)){
              *m_pSolverLog <<"Lambda converged"<<std::endl;
            }
#endif

            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                nodeData.m_u1BufferPtr->m_front = nodeData.m_u1BufferPtr->m_front + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

               #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"Node: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);
               #endif


                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InVelocityLocal) {
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InEnergyLocalMix){
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache1,
                                                                          nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                                          nodeData.m_LambdaBack,
                                                                          nodeData.m_LambdaFront,
                                                                          nodeData.m_G_ii,
                                                                          m_Settings.m_AbsTol,
                                                                          m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }

            }
            // SECOND BODY
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;
                nodeData.m_u2BufferPtr->m_front = nodeData.m_u2BufferPtr->m_front  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InVelocityLocal) {
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                                  nodeData.m_u2BufferPtr->m_front,
                                                                  m_Settings.m_AbsTol,
                                                                  m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InEnergyLocalMix){
                    if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                                          nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                                          nodeData.m_LambdaBack,
                                                                          nodeData.m_LambdaFront,
                                                                          nodeData.m_G_ii,
                                                                          m_Settings.m_AbsTol,
                                                                          m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }
            }

            if(m_Settings.m_eConvergenceMethod == InclusionSolverSettings::InLambda) {
                if(m_iterationsNeeded >= m_Settings.m_MinIter && m_bConverged) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                    if(!nodeData.m_bConverged) {
                        //converged stays false;
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }

            // Swap Lambdas, but dont swap Velocities...
            // Swap velocities when we finished ONE Sor Prox Iteration! (very important!)
            nodeData.swapLambdas();


        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettings & m_Settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_iterationsNeeded; ///< Access to global iteration counter

};


/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxStepSplitNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef TContactGraph ContactGraphType;

    typedef typename ContactGraphType::NodeDataType NodeDataType;
    typedef typename ContactGraphType::SplitBodyNodeType NodeType;

    SorProxStepSplitNodeVisitor(const InclusionSolverSettings &settings, bool & globalConverged, const unsigned int & globalIterationNeeded):
                           m_Settings(settings),m_bConverged(globalConverged),
                           m_iterationsNeeded(globalIterationNeeded)
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    void visitNode(NodeType& node){
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */

        // Calculate the exact values for the billateral split nodes

        // Build gamma = [u1-u2, u2-u3, u3-u4,..., un-1- un]
        // make all but first
        for(unsigned int i = 0; i < node.m_partRanks.size()-1; i++){
            node.m_gamma.segment<NDOFuObj>(NDOFuObj*(i+1)) =   node.m_u_G.segment<NDOFuObj>(NDOFuObj*i)
                                                             - node.m_u_G.segment<NDOFuObj>(NDOFuObj*(i+1));
        }
        // make first entry in gamma
        //node.m_gamma.head<NDOFuObj>() = node.m_pBody->m_pSolverData->m_uBuffer.m_front - node.m_u_G.head<NDOFuObj>();

        // calculate L⁻¹*gamma = Lambda, where L⁻¹ is the matrix choosen by the multiplicity


    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettings & m_Settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_iterationsNeeded; ///< Access to global iteration counter

};


/**
@brief Visitor for class ContactGraph
*/
template<typename TContactGraph>
class SorProxInitNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef TContactGraph ContactGraphType;
    typedef typename ContactGraphType::NodeDataType NodeDataType;
    typedef typename ContactGraphType::EdgeDataType EdgeDataType;
    typedef typename ContactGraphType::EdgeType EdgeType;
    typedef typename ContactGraphType::NodeType NodeType;

    SorProxInitNodeVisitor()
    {}

    void setLog(Logging::Log * solverLog){
        m_pSolverLog = solverLog;
    }

    // Set Sor Prox parameter
    void setParams(PREC alpha){
        m_alpha = alpha;
    }

    void visitNode(NodeType & node){

        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;

        // Get lambda from percussion pool otherwise set to zero
        // TODO
        nodeData.m_LambdaBack.setZero();

        // (1+e)*xi -> b
        nodeData.m_b = nodeData.m_I_plus_eps.asDiagonal() * nodeData.m_xi;

        // u_0 , calculate const b
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED) {

            // m_back contains u_s + M^⁻1*h*deltaT already!
            // add + initial values M^⁻1 W lambda0 from percussion pool
            nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack );


            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {

            // m_back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u1BufferPtr->m_back;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }



        // Calculate R_ii
        nodeData.m_R_i_inv_diag(0) = m_alpha / (nodeData.m_G_ii(0,0));
        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().template tail<2>()).maxCoeff());
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, " nodeData.m_b :"<< nodeData.m_b.transpose() <<std::endl
                << " nodeData.m_G_ii :"<<std::endl<< nodeData.m_G_ii <<std::endl
                << " nodeData.m_R_i_inv_diag: "<< nodeData.m_R_i_inv_diag.transpose() <<std::endl;);
        #endif

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog,  " nodeData.m_mu: "<< nodeData.m_mu <<std::endl;);
        #endif


    }

private:
    Logging::Log * m_pSolverLog;
    PREC m_alpha;
};




#endif

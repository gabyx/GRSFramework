#ifndef GMSF_Dynamics_Inclusion_ContactGraph_hpp
#define GMSF_Dynamics_Inclusion_ContactGraph_hpp


//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

/** Contact Graph */
#include "GMSF/Common/TypeDefs.hpp"
#include "GMSF/Common/LogDefines.hpp"
#include "GMSF/Common/AssertionDebug.hpp"

#include "GMSF/Common/BitCount.hpp"
#include "GMSF/Common/BitRepresentation.hpp"
#include "GMSF/Common/EnumClassHelper.hpp"

#include "GMSF/Dynamics/Inclusion/GeneralGraph.hpp"
#include "GMSF/Dynamics/Collision/CollisionData.hpp"
#include "GMSF/Dynamics/Inclusion/ContactParameterMap.hpp"

#include "GMSF/Dynamics/Inclusion/ContactModels.hpp"
#include "GMSF/Dynamics/Inclusion/ContactGraphNodeData.hpp"

#include "GMSF/Dynamics/Inclusion/ContactGraphVisitors.hpp"



/** Contact Graph which is no meant to be used to iterate over the noced */
class ContactGraph: public Graph::GeneralGraph< ContactGraphNodeData,ContactGraphEdgeData > {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    using NodeDataType = ContactGraphNodeData;
    using EdgeDataType = ContactGraphEdgeData;
    using EdgeType = typename Graph::Edge< NodeDataType, EdgeDataType>;
    using NodeType = typename Graph::Node< NodeDataType, EdgeDataType>;

    using NodeListType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType;
    using EdgeListType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType;
    using NodeListIteratorType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType;
    using EdgeListIteratorType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType;

public:

    ContactGraph(ContactParameterMap * contactParameterMap){
        m_pContactParameterMap = contactParameterMap;
    }
    ~ContactGraph() {
        clearGraph();
    }

    void clearGraph() {
        // This deletes all nodes, edges, and decrements the reference counts for the nodedata and edgedata
        // cleanup allocated memory
        for(auto n_it = this->m_nodes.begin(); n_it != this->m_nodes.end(); ++n_it)
            delete (*n_it);
        for(auto e_it = this->m_edges.begin(); e_it != this->m_edges.end(); ++e_it)
            delete (*e_it);
        //cout << "clear graph"<<endl;
        this->m_nodes.clear();
        m_nodeCounter = 0;
        this->m_edges.clear();
        m_edgeCounter = 0;
        m_simBodiesToContactsMap.clear();

        m_nLambdas = 0;

        m_usedContactModels = 0;
    }

    void addNode(CollisionData * pCollData) {

        ASSERTMSG(pCollData->m_pBody1 != nullptr && pCollData->m_pBody2 != nullptr, " Bodys are null pointers?");
        //cout << "add node : "<<m_nodeCounter<< " body id:" << pCollData->m_pBody1->m_id <<" and "<< pCollData->m_pBody2->m_id <<endl;

        //  add a contact node to the graph
        // check to which nodes we need to connect?
        // all nodes (contacts) which are in the BodyContactList (maps bodies -> contacts)

        // add the pNodeData to the node list
        this->m_nodes.push_back( new NodeType(m_nodeCounter));
        NodeType * addedNode = this->m_nodes.back();
        addedNode->m_nodeData.m_pCollData = pCollData;

        // Compute general parameters for the contact
        setContactModel(addedNode->m_nodeData);

        // FIRST BODY!
        if( pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

            computeW<1>( addedNode->m_nodeData);
            connectNode<1>( addedNode);

        } else if( pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }


        // SECOND BODY!
        if( pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

            computeW<2>( addedNode->m_nodeData);
            connectNode<2>( addedNode);

        } else if( pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }


        m_nodeCounter++;
    }

    static const MatrixUBodyDyn & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }
    static const MatrixUBodyDyn * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::unordered_map<const RigidBodyType *, NodeListType > m_simBodiesToContactsMap;
    typedef typename std::unordered_map<const RigidBodyType *, NodeListType >::iterator  BodyToContactsListIterator;

    unsigned int getNLambdas(){return m_nLambdas;}

    unsigned int getNContactModelsUsed(){
        return BitCount::count(m_usedContactModels);
    }
private:
    using ContactModelEnumIntType = typename std::underlying_type<ContactModels::Enum>::type;
    ContactModelEnumIntType m_usedContactModels = 0; ///< Bitflags which mark all used contactmodels

    unsigned int m_nLambdas = 0; ///< How many forces we have counted over all nodes

    void setContactModel(NodeDataType & nodeData) {

        // Specify the contact model
        nodeData.m_contactParameter  = m_pContactParameterMap->getContactParams(nodeData.m_pCollData->m_pBody1->m_eMaterial,
                                       nodeData.m_pCollData->m_pBody2->m_eMaterial);

        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ) {
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            // Set flag for the corresponding model
            m_usedContactModels |= 1 << EnumConversion::toIntegral(ContactModels::Enum::UCF);

            const unsigned int dimSet = CONTACTMODELTYPE(ContactModels::Enum::UCF)::ConvexSet::Dimension;
            m_nLambdas += dimSet; // Add to counter

            //Set all the matrix sizes ========================================================================
            nodeData.m_W_body1.setZero(NDOFuBody, ContactModels::getLambdaDim(ContactModels::Enum::UCF));
            nodeData.m_W_body2.setZero(NDOFuBody, ContactModels::getLambdaDim(ContactModels::Enum::UCF));

            nodeData.m_eps.setZero(dimSet);
            nodeData.m_chi.setZero(dimSet);

            // ==============================================================================================

            nodeData.m_eps(0) = nodeData.m_contactParameter.m_params[CMT::epsNIdx];
            nodeData.m_eps(1) = nodeData.m_contactParameter.m_params[CMT::epsTIdx];
            nodeData.m_eps(2) = nodeData.m_eps(1);

        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }
    }

    template<int bodyNr>
    void computeW(NodeDataType & nodeData) {



        if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF) {


            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData * pCollData;

            pCollData = nodeData.m_pCollData;



            if(bodyNr == 1) {
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
        //cout << "add self edge: "<<pNode->m_nodeNumber<<" to "<<pNode->m_nodeNumber<<" body Id:"<< pBody->m_id<<endl;
        // ===========================================================================

        // Get all contacts on this body and connect to them =========================
        NodeListType & nodeList = m_simBodiesToContactsMap[pBody];
        //iterate over the nodeList and add edges!
        typename NodeListType::iterator it;
        // if no contacts are already on the body we skip this
        for(it = nodeList.begin(); it != nodeList.end(); ++it) {

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
            //cout << "add edge: "<<pNode->m_nodeNumber<<" to "<<(*it)->m_nodeNumber<<" body Id:"<< pBody->m_id<<endl;
        }

        // Add new Node to the list;
        nodeList.push_back(pNode);

    }

    ContactParameterMap* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter = 0; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter = 0; ///< An edge counter, starting at 0.

};

/** Contact Graph which can be used to iterate over the nodes */
// #define RESIDUAL_SORTED_ITERATION   ///< Define this macro here to get a residual sorted iteration for SOR_CONTACT
class ContactGraphIteration: public Graph::GeneralGraph< ContactGraphNodeDataIteration,ContactGraphEdgeData > {
public:


    DEFINE_RIGIDBODY_CONFIG_TYPES

    using NodeDataType = ContactGraphNodeDataIteration;
    using EdgeDataType = ContactGraphEdgeData;
    using NodeType = typename Graph::Node< NodeDataType, EdgeDataType>;
    using EdgeType = typename Graph::Edge< NodeDataType, EdgeDataType>;

    using NodeListType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType;
    using EdgeListType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType;
    using NodeListIteratorType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType;
    using EdgeListIteratorType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType;

    ContactGraphIteration(ContactParameterMap * contactParameterMap)
        #ifdef RESIDUAL_SORTED_ITERATION
        :m_nodesResSorted1(cFunc),m_nodesResSorted2(cFunc)
        #endif // RESIDUAL_SORTED_ITERATION
    {
        m_pContactParameterMap = contactParameterMap;

         #ifdef RESIDUAL_SORTED_ITERATION
        // Init front/back
        m_nodesBackRes = &m_nodesResSorted1;
        m_nodesFrontRes = &m_nodesResSorted2;
        #endif // RESIDUAL_SORTED_ITERATION
    }

    ~ContactGraphIteration() {
        clearGraph();
    }

    void initForIteration() {

        #ifdef RESIDUAL_SORTED_ITERATION
        m_nodesBackRes->clear();
        m_nodesFrontRes->clear();
        #endif // RESIDUAL_SORTED_ITERATION

        m_firstIteration = true;
        m_maxResidual = 0;
    }

    template<typename TNodeVisitor>
	void applyNodeVisitorSpecial(TNodeVisitor & vv){

    #ifdef RESIDUAL_SORTED_ITERATION
    if( m_firstIteration ){
        for(auto curr_node = this->m_nodes.begin(); curr_node != this->m_nodes.end(); curr_node++){
            vv.visitNode(*(*curr_node));
        }
        m_firstIteration = false;
    }
    else{
        for(auto curr_node = m_nodesBackRes->begin(); curr_node != m_nodesBackRes->end(); curr_node++){
            vv.visitNode(*curr_node->second);
        }
    }
    #else // No residual sorted iteration!
    for(auto curr_node = this->m_nodes.begin(); curr_node != this->m_nodes.end(); curr_node++){
        vv.visitNode(*(*curr_node));
    }
    #endif // RESIDUAL_SORTED_ITERATION

	}

    void resetAfterOneIteration(unsigned int globalIterationCounter){

        #ifdef RESIDUAL_SORTED_ITERATION
        // Switch potiner of residual list;
        if( globalIterationCounter % 1 == 0 ){
            auto * t = m_nodesBackRes;
            m_nodesBackRes = m_nodesFrontRes;
            m_nodesFrontRes = t;
        }
        // Clear front
        m_nodesFrontRes->clear();
        #endif // RESIDUAL_SORTED_ITERATION

        m_maxResidual = 0;
    }

    void clearGraph() {
        // This deletes all nodes, edges, and decrements the reference counts for the nodedata and edgedata
        // cleanup allocated memory
        deleteNodesAndEdges();

        m_edgeCounter = 0;
        m_nodeCounter = 0;

        m_simBodiesToContactsMap.clear();

        m_usedContactModels = 0;
        m_maxResidual = 0.0;
    }

    template<bool addEdges = true>
    void addNode(CollisionData * pCollData) {

        ASSERTMSG(pCollData->m_pBody1 != nullptr && pCollData->m_pBody2 != nullptr, " Bodys are null pointers?");
        //cout << "add node : "<<m_nodeCounter<< " body id:" << RigidBodyId::getBodyIdString(pCollData->m_pBody1) <<" and "<< RigidBodyId::getBodyIdString(pCollData->m_pBody2) <<endl;
//        std::cout <<"b1:" << pCollData->m_pBody1->m_q_KI<< std::endl << pCollData->m_pBody1->m_r_S<< std::endl;;
//        std::cout <<"b2:" << pCollData->m_pBody2->m_q_KI << std::endl << pCollData->m_pBody2->m_r_S<< std::endl;;;
        //  add a contact node to the graph
        // check to which nodes we need to connect?
        // all nodes (contacts) which are in the BodyContactList (maps bodies -> contacts)

        // add the pNodeData to the node list
        this->m_nodes.push_back( new NodeType(m_nodeCounter));
        NodeType * addedNode = this->m_nodes.back();
        addedNode->m_nodeData.m_pCollData = pCollData;

        initNode<addEdges>(addedNode);


        m_nodeCounter++;
    }


    static const MatrixUBodyDyn & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const MatrixUBodyDyn * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::unordered_map<const RigidBodyType *, NodeListType > m_simBodiesToContactsMap;

    PREC m_maxResidual = 0.0;

    unsigned int getNContactModelsUsed(){
        return BitCount::count<ContactModelEnumIntType>(m_usedContactModels);
    }

private:
    using ContactModelEnumIntType = typename std::underlying_type<ContactModels::Enum>::type;
    ContactModelEnumIntType m_usedContactModels; ///< Bitflags which mark all used contactmodels

    bool m_firstIteration = true;

    void setContactModel(NodeDataType & nodeData) {

        // Specify the contact model
        nodeData.m_contactParameter  = m_pContactParameterMap->getContactParams(nodeData.m_pCollData->m_pBody1->m_eMaterial,
                                                                                nodeData.m_pCollData->m_pBody2->m_eMaterial);

        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF  ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD
          ) {
            // Set flag for the corresponding model
            m_usedContactModels |= 1 << EnumConversion::toIntegral(nodeData.m_contactParameter.m_contactModel);

            const unsigned int dimSet = ContactModels::getLambdaDim(ContactModels::Enum::UCF);
            nodeData.m_eps.setZero(dimSet);
            nodeData.m_chi.setZero(dimSet);

            // Set epsilon  values
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            nodeData.m_eps(0) = nodeData.m_contactParameter.m_params[CMT::epsNIdx];
            nodeData.m_eps(1) = nodeData.m_contactParameter.m_params[CMT::epsTIdx];
            nodeData.m_eps(2) = nodeData.m_eps(1);

        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }

    }

    template<bool addEdges = true>
    inline void initNode(NodeType * pNode){
        // Compute general parameters for the contact
        setContactModel(pNode->m_nodeData);

        // FIRST BODY!
        RigidBodyType * pBody = pNode->m_nodeData.m_pCollData->m_pBody1;
        if( pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
            initNodeSimBody<1,addEdges>(pNode,pBody);
        }else if(pBody->m_eMode == RigidBodyType::BodyMode::ANIMATED ){
            ERRORMSG("ContactGraph:: Animated body, node init not implemented")
        }

        // SECOND BODY!
        pBody = pNode->m_nodeData.m_pCollData->m_pBody2;
        if( pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
           initNodeSimBody<2,addEdges>(pNode,pBody);
        }else if(pBody->m_eMode == RigidBodyType::BodyMode::ANIMATED ){
           ERRORMSG("ContactGraph:: Animated body, node init not implemented")
        }
    }

    template<int bodyNr, bool addEdges = true>
    inline void initNodeSimBody(NodeType * pNode, RigidBodyType * pBody) {

        // Get all contacts on this body and connect to them (later) =========================
        NodeListType & nodeList = m_simBodiesToContactsMap[pBody];

        //Set Flag that this Body is in ContactGraph
        pBody->m_pSolverData->m_bInContactGraph = true;
        // Unset the flag when this Node is removed;

        //Link to FrontBackBuffer
        if(bodyNr==1){
            pNode->m_nodeData.m_u1BufferPtr = & pBody->m_pSolverData->m_uBuffer;
        }else{
            pNode->m_nodeData.m_u2BufferPtr = & pBody->m_pSolverData->m_uBuffer;
        }


        if( addEdges ){
            // Add self edge! ===========================================================
            EdgeType * addedEdge;
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

            // iterate over the nodeList and add edges!
            // if no contacts are already on the body we skip this
            auto itEnd = nodeList.end();
            for(auto it = nodeList.begin(); it != itEnd; ++it) {

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
        }

        // Add new Node to the list;
        nodeList.push_back(pNode);

    }

    ContactParameterMap* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter = 0; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter = 0; ///< An edge counter, starting at 0.


    #ifdef RESIDUAL_SORTED_ITERATION
    /** Sorted Nodes by Max Residual ==========================================================
    * This list is currently not used, because test showed that it does not improve convergence
    */
    using CType = std::function< bool(std::pair<PREC, NodeType *> const&, std::pair<PREC, NodeType *> const&)>;

    CType cFunc = [](std::pair<PREC, NodeType *> const& a,
                     std::pair<PREC, NodeType *> const& b)->bool{
        return a.first < b.first;
    };

    typedef std::multiset<std::pair<PREC, NodeType *>,CType> NodeListTypeResidualSorted;
    NodeListTypeResidualSorted * m_nodesBackRes;  // The list on which we are iterating
    NodeListTypeResidualSorted * m_nodesFrontRes; // The list on which we are inserting (sorted)
    NodeListTypeResidualSorted m_nodesResSorted1;
    NodeListTypeResidualSorted m_nodesResSorted2;
    /** ========================================================================================= */
    #endif
};





#endif

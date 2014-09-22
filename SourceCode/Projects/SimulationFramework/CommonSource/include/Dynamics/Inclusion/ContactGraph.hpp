#ifndef ContactGraph_hpp
#define ContactGraph_hpp


//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

/** Contact Graph */
#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "BitCount.hpp"
#include "BitRepresentation.hpp"
#include "EnumClassHelper.hpp"

#include "GeneralGraph.hpp"
#include "CollisionData.hpp"
#include "ContactModels.hpp"
#include "ContactParameterMap.hpp"
#include "ContactGraphNodeData.hpp"

#include InclusionSolverSettings_INCLUDE_FILE

#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"



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

    ContactGraph(ContactParameterMap * contactParameterMap):
        m_nodeCounter(0),m_edgeCounter(0),m_nLambdas(0) {
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

    static const Eigen::Matrix<PREC,6,3> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }
    static const Eigen::Matrix<PREC,6,3> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
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
    ContactModelEnumIntType m_usedContactModels; ///< Bitflags which mark all used contactmodels

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

            //nodeData.m_chi.setZero(dimSet);
            //nodeData.m_eps.setZero(dimSet);

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
                //nodeData.m_W_body1.setZero());

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
                //nodeData.m_W_body2.setZero());

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

    ContactGraphIteration(ContactParameterMap * contactParameterMap):
        m_nodeCounter(0),m_edgeCounter(0)
        #ifdef RESIDUAL_SORTED_ITERATION
        ,m_nodesResSorted1(cFunc),m_nodesResSorted2(cFunc)
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


    static const Eigen::Matrix<PREC,6,3> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const Eigen::Matrix<PREC,6,3> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::unordered_map<const RigidBodyType *, NodeListType > m_simBodiesToContactsMap;

    PREC m_maxResidual;

    unsigned int getNContactModelsUsed(){
        return BitCount::count<ContactModelEnumIntType>(m_usedContactModels);
    }

private:
    using ContactModelEnumIntType = typename std::underlying_type<ContactModels::Enum>::type;
    ContactModelEnumIntType m_usedContactModels; ///< Bitflags which mark all used contactmodels

    bool m_firstIteration;

    friend class ContactSorProxStepNodeVisitor;
    friend class FullSorProxStepNodeVisitor;

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

            //Set the minmial stuff
            const unsigned int dimSet = ContactModels::getLambdaDim(ContactModels::Enum::UCF);
            //nodeData.m_eps.setZero();
            //nodeData.m_chi.setZero();

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

    unsigned int m_nodeCounter; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter; ///< An edge counter, starting at 0.


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


/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
class SorProxStepNodeVisitor {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = ContactGraphIteration;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;


    SorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                           bool & globalConverged,
                           const unsigned int & globalIterationNeeded,
                           ContactGraphType * graph):
        m_settings(settings),m_bConverged(globalConverged),
        m_globalIterationCounter(globalIterationNeeded),
        m_pGraph(graph)
    {}


    inline void visitNode(NodeType & node) {
        m_delegate(node);
    }

    void setLog(Logging::Log * solverLog) {
        m_pSolverLog = solverLog;
    }
    // Set Sor Prox parameter, before calling visitNode
    void setParams(PREC alpha) {
        m_alpha = alpha;
    }

//    template<int B>
//    void doVelocityUpdate(typename ContactGraphType::NodeDataType & nodeData) {
//        typedef decltype(nodeData.m_u1BufferPtr->m_front) VectorUType;
//        RigidBodyType * pBody;
//        VectorUType * pUBuffer;
//
//        if(B==1) {
//            pUBuffer = &nodeData.m_u1BufferPtr->m_front;
//            pBody = nodeData.m_pCollData->m_pBody1;
//        } else {
//            pUBuffer = &nodeData.m_u2BufferPtr->m_front;
//            pBody = nodeData.m_pCollData->m_pBody2;
//        }
//
//        // u_S + Minv *h * deltaT
//        *pUBuffer = pBody->m_pSolverData->m_uBegin + pBody->m_MassMatrixInv_diag.asDiagonal() * pBody->m_h_term * m_settings.m_deltaT;
//
//        // Iterate over all nodes and add contribution
//        auto nodeList = m_pGraph->m_simBodiesToContactsMap[pBody];
//        for(auto it = nodeList.begin(); it!=nodeList.end(); ++it) {
//            *pUBuffer += pBody->m_MassMatrixInv_diag.asDiagonal() * ContactGraphType::getW_bodyRef((*it)->m_nodeData,pBody) * (*it)->m_nodeData.m_LambdaFront;
//        }
//
//    }

    void setNewDamping(typename ContactGraphType::NodeDataType & nodeData) {
        using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
        nodeData.m_contactParameter.m_params[CMT::d_NIdx] = 1e-6;
        nodeData.m_contactParameter.m_params[CMT::d_TIdx] = 1e-6;
        recalculateR(nodeData, nodeData.m_contactParameter);
    }

    void recalculateR(typename ContactGraphType::NodeDataType & nodeData, ContactParameter & contactParameter) {

        nodeData.m_G_ii.setZero();
        if(nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }




        if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
            Vector3 dinv(contactParameter.m_params[CMT::d_NIdx], //d_N
                         contactParameter.m_params[CMT::d_TIdx], //d_T
                         contactParameter.m_params[CMT::d_TIdx]); //d_T
            nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*dinv;
        } else if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD) {
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFDD);
            Vector3 dinv;
            dinv(0) = contactParameter.m_params[CMT::d_NIdx];
            // if lambda_N <= eps, set damping to d_Tfix
            if( std::abs(nodeData.m_LambdaBack(0)) <= contactParameter.m_params[CMT::epsIdx] ) {
                dinv.tail<2>().setConstant( contactParameter.m_params[CMT::d_TfixIdx] );
            } else { //dinvT = gammaMax / (mu *lambdaN)
                dinv.tail<2>().setConstant( contactParameter.m_params[CMT::gamma_maxIdx] / (contactParameter.m_params[CMT::muIdx]*nodeData.m_LambdaBack(0) ) );
            }
            nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*dinv;
        }

        // Calculate R_ii
        // Take also offdiagonal values for lambda_N
        //nodeData.m_R_i_inv_diag(0) = m_alpha / std::max(std::max(nodeData.m_G_ii(0,0), nodeData.m_mu(0)*nodeData.m_G_ii(0,1)), nodeData.m_mu(0)*nodeData.m_G_ii(0,2));
        // Take only diagonal
        nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);
        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().tail<2>()).maxCoeff());
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;
    }


    // Function delegate to make class visitable
    typedef srutil::delegate1<void,NodeType &> VisitNodeDelegate;
protected:
    Logging::Log * m_pSolverLog;
    PREC m_alpha;
    const InclusionSolverSettingsType & m_settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_globalIterationCounter; ///< Access to global iteration counter

    ContactGraphType * m_pGraph;


    VisitNodeDelegate m_delegate;

};

/**
* This is a contact sor, projects one contact together!
*/
class ContactSorProxStepNodeVisitor : public SorProxStepNodeVisitor {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = ContactGraphIteration;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;


    ContactSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                                  bool & globalConverged,
                                  const unsigned int & globalIterationNeeded,
                                  ContactGraphType * graph):
        SorProxStepNodeVisitor(settings,globalConverged,globalIterationNeeded,graph)
    {
        this->m_delegate = VisitNodeDelegate::from_method<ContactSorProxStepNodeVisitor,
              &ContactSorProxStepNodeVisitor::visitNode>(this);
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD  ) {

            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);

            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body2.transpose() * nodeData.m_u2BufferPtr->m_front;
            }

            // Experimental
            //Relaxation term damper (take care R_i_inv needs to be initialized as well!)
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                //Drive damping to zero after some iterations:
                //if (m_globalIterationCounter == 300) {
                    //nodeData.m_contactParameter.m_params[4] = 1e-7;
                //}

                nodeData.m_LambdaFront(0) += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
                nodeData.m_LambdaFront(1) += nodeData.m_LambdaBack(1) * nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT;
                nodeData.m_LambdaFront(2) += nodeData.m_LambdaBack(2) * nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT;


            } else if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFDD);
                recalculateR(nodeData,nodeData.m_contactParameter);
                // if lambda_N <= eps, set damping to d_Tfix
                PREC dinvT;
                if( std::abs(nodeData.m_LambdaBack(0)) <= nodeData.m_contactParameter.m_params[CMT::epsIdx] ) {
                    dinvT = nodeData.m_contactParameter.m_params[CMT::d_TfixIdx];
                } else { //dinvT = gammaMax / (mu *lambdaN)
                    dinvT = nodeData.m_contactParameter.m_params[CMT::gamma_maxIdx] / (nodeData.m_contactParameter.m_params[CMT::muIdx]*nodeData.m_LambdaBack(0) );
                }

                nodeData.m_LambdaFront(0) += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
                nodeData.m_LambdaFront(1) += nodeData.m_LambdaBack(1) * dinvT / m_settings.m_deltaT;
                nodeData.m_LambdaFront(2) += nodeData.m_LambdaBack(2) * dinvT / m_settings.m_deltaT;
            }

            //Prox
            // HACK drive up parameter mu, does not help
//            PREC mu =  nodeData.m_mu(0);
//            if(m_globalIterationCounter <= 10){
//                 mu = ((m_globalIterationCounter) / 10 ) * nodeData.m_mu(0);
//                 m_bConverged = false;
//            }
//            else if (m_globalIterationCounter <= 300) {
//                mu = ((m_globalIterationCounter-200) / 500 ) * nodeData.m_mu(0);
//                m_bConverged = false;
//            }
//            else{
//                mu =  nodeData.m_mu(0);
//            }

            // PROX  prox(lambda - R_i_inv * gamma) ==================================================================================
            if(m_settings.m_eMethod == InclusionSolverSettingsType::SOR_CONTACT_DS){
                // De Saxe Formulation
                // add correction term mu*gammaT to gammaN (for DeSaxce Cone Formulation)
                nodeData.m_LambdaFront(0) += nodeData.m_contactParameter.m_params[CMT::muIdx]*nodeData.m_LambdaFront.tail<2>().norm();
                //Multiply R_i_inverse
                nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
                nodeData.m_LambdaFront += nodeData.m_LambdaBack;
                //Prox onto friction cone
                Prox::ProxFunction<ConvexSets::Cone3D>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx],nodeData.m_LambdaFront);
            }else{
                // Alart Curnier Formulation
                //Multiply R_i_inverse
                nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
                nodeData.m_LambdaFront += nodeData.m_LambdaBack;
                //Prox onto R+ and Disk
                Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx],nodeData.m_LambdaFront);
            }


            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)) {
                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> Lambda converged"<<std::endl);
            }

            // Velocity Updates ====================================================================================================================
            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            decltype(nodeData.m_LambdaFront) deltaLambda = nodeData.m_LambdaFront - nodeData.m_LambdaBack ;
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;

                // Velocity update (wahrscheinlich Auslöschung bei Lambda)
                nodeData.m_u1BufferPtr->m_front = uCache1
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * deltaLambda;

                //Sepcial update (no differences)
                //doVelocityUpdate<1>(nodeData);


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);


                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(      uCache1,
                                                 nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
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
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;

                // Velocity update (wahrscheinlich Auslöschung bei Lambda)
                nodeData.m_u2BufferPtr->m_front = uCache2
                                                  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 * deltaLambda ;

                //Sepcial update (no differences)
                //doVelocityUpdate<2>(nodeData);

                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);

                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                 nodeData.m_u2BufferPtr->m_front,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                 nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
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


            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) ) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,
                                            nodeData.m_LambdaFront,
                                            m_settings.m_AbsTol,
                                            m_settings.m_RelTol,
                                            residual
                                                                         );
                    m_pGraph->m_maxResidual = std::max(residual,m_pGraph->m_maxResidual);
                    if(!nodeData.m_bConverged) {
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }


            // Save residual and insert into front buffer
            #ifdef RESIDUAL_SORTED_ITERATION
                m_pGraph->m_nodesFrontRes->insert(std::make_pair(residual,&node));
            #endif // RESIDUAL_SORTED_ITERATION

            // Swap Lambdas, but dont swap Velocities...
            //nodeData.m_LambdaBack = nodeData.m_LambdaFront; // necessary if we use doVelocityUpdate function!
            nodeData.swapLambdas(); // faster only switch pointers



        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

};


/**
* This is a full sor, projects normal and then tangential of one contact consecutive!
*/
class FullSorProxStepNodeVisitor : public SorProxStepNodeVisitor {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = ContactGraphIteration;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;

    FullSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                               bool & globalConverged,
                               const unsigned int & globalIterationNeeded,
                               ContactGraphType * graph):
        SorProxStepNodeVisitor(settings,globalConverged,globalIterationNeeded,graph) {
        this->m_delegate = VisitNodeDelegate::from_method<  FullSorProxStepNodeVisitor,
              &FullSorProxStepNodeVisitor::visitNode>(this);
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ) {

            //First normal direction ===================================

            PREC lambda_N = nodeData.m_b(0);
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                lambda_N += nodeData.m_W_body1.transpose().row(0) * uCache1 ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;
                lambda_N += nodeData.m_W_body2.transpose().row(0) * uCache2;
            }

            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_N += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
            }
            lambda_N = -nodeData.m_R_i_inv_diag(0) * lambda_N;
            lambda_N += nodeData.m_LambdaBack(0);

            //Prox
            Prox::ProxFunction<ConvexSets::RPlus>::doProxSingle( lambda_N, lambda_N );

            //Apply to bodies
            nodeData.m_LambdaFront(0) = lambda_N;
            PREC deltaLambda_N = lambda_N - nodeData.m_LambdaBack(0); // Delta lambda_N

            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u1BufferPtr->m_front = uCache1
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body1.col(0) * deltaLambda_N;
            }
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u2BufferPtr->m_front = uCache2
                                                  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body2.col(0) * deltaLambda_N;
            }


            // =========================================================

            // Second Tangential direction =============================
            Vector2 lambda_T = nodeData.m_b.tail<2>();
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body1.transpose().bottomRows<2>() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body2.transpose().bottomRows<2>() * nodeData.m_u2BufferPtr->m_front;
            }
            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                 using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_T += nodeData.m_LambdaBack.tail<2>() * (nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT);
            }

            lambda_T = - (nodeData.m_R_i_inv_diag.tail<2>().asDiagonal() * lambda_T).eval();
            lambda_T += nodeData.m_LambdaBack.tail<2>();

            //Prox
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
            Prox::ProxFunction<ConvexSets::Disk>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx] * lambda_N, lambda_T );

            nodeData.m_LambdaFront.tail<2>() =  lambda_T;

            lambda_T = lambda_T - nodeData.m_LambdaBack.tail<2>(); // Delta lambda_T



            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u1BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1.rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);

                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(      uCache1,
                                                 nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
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
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u2BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal()*nodeData.m_W_body2.rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);


                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                 nodeData.m_u2BufferPtr->m_front,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                 nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
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
            // =========================================================



            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)) {
                LOGSLLEVEL3_CONTACT(m_pSolverLog , "\t---> Lambda converged"<<std::endl);
            }




            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) ) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,
                                            nodeData.m_LambdaFront,
                                            m_settings.m_AbsTol,
                                            m_settings.m_RelTol,
                                            residual
                                                                         );
                    m_pGraph->m_maxResidual = std::max(residual,m_pGraph->m_maxResidual);
                    if(!nodeData.m_bConverged) {
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }


            // Swap Lambdas, but dont swap Velocities...
            //nodeData.m_LambdaBack = nodeData.m_LambdaFront; // necessary if we use doVelocityUpdate function!
            nodeData.swapLambdas(); // faster only switch pointers



        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

};

/**
* This is a normal SOR Prox, projects normal directions of one contact consecutive!
*/
class NormalSorProxStepNodeVisitor : public SorProxStepNodeVisitor {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = ContactGraphIteration;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;

    NormalSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                               bool & globalConverged,
                               const unsigned int & globalIterationNeeded,
                               ContactGraphType * graph):
        SorProxStepNodeVisitor(settings,globalConverged,globalIterationNeeded,graph) {
        this->m_delegate = VisitNodeDelegate::from_method<  NormalSorProxStepNodeVisitor,
              &NormalSorProxStepNodeVisitor::visitNode >(this);
    }

    void setLastUpdate(bool b){
        m_lastUpdate = b;
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ) {

            //Only normal direction ===================================

            PREC lambda_N = nodeData.m_b(0);
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                lambda_N += nodeData.m_W_body1.transpose().row(0) * uCache1 ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                uCache2 = nodeData.m_u2BufferPtr->m_front;
                lambda_N += nodeData.m_W_body2.transpose().row(0) * uCache2;
            }

            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_N += nodeData.m_LambdaBack(0) * nodeData.m_contactParameter.m_params[CMT::d_NIdx] / m_settings.m_deltaT;
            }
            lambda_N = -nodeData.m_R_i_inv_diag(0) * lambda_N;
            lambda_N += nodeData.m_LambdaBack(0);

            //Prox
            Prox::ProxFunction<ConvexSets::RPlus>::doProxSingle( lambda_N, lambda_N );

            //Apply to bodies

            PREC deltaLambda_N = lambda_N - nodeData.m_LambdaBack(0); // Delta lambda_N

            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u1BufferPtr->m_front = uCache1
                                                  + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body1.col(0) * deltaLambda_N;
            }
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                nodeData.m_u2BufferPtr->m_front = uCache2
                                                  + nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() *
                                                  nodeData.m_W_body2.col(0) * deltaLambda_N;
            }

            if(m_lastUpdate){
                nodeData.m_LambdaFront(0) = lambda_N;
            }else{
                nodeData.m_LambdaBack(0) = lambda_N;
            }
            // =========================================================

            // Apply the tangential step node visitor now

        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

    bool m_lastUpdate = false;

};

/**
* This is a tangential SOR Prox, projects tangential directions of one contact consecutive!
*/
class TangentialSorProxStepNodeVisitor : public SorProxStepNodeVisitor {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = ContactGraphIteration;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;

    TangentialSorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                               bool & globalConverged,
                               const unsigned int & globalIterationNeeded,
                               ContactGraphType * graph):
        SorProxStepNodeVisitor(settings,globalConverged,globalIterationNeeded,graph) {
        this->m_delegate = VisitNodeDelegate::from_method<  TangentialSorProxStepNodeVisitor,
              &TangentialSorProxStepNodeVisitor::visitNode>(this);
    }

    void visitNode(NodeType& node) {
        /* Convergence Criterias are no more checked if the m_bConverged (gloablConverged) flag is already false
           Then the iteration is not converged somewhere, and we need to wait till the next iteration!
        */
        typename ContactGraphType::NodeDataType & nodeData = node.m_nodeData;
        static VectorUBody uCache1,uCache2;
        PREC residual;


        LOGSLLEVEL3_CONTACT(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
        if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED) {
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
        }


        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
                nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ) {


            // Second Tangential direction =============================
            Vector2 lambda_T = nodeData.m_b.tail<2>();
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body1.transpose().bottomRows<2>() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                lambda_T += nodeData.m_W_body2.transpose().bottomRows<2>() * nodeData.m_u2BufferPtr->m_front;
            }
            // Damping
            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                 using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                lambda_T += nodeData.m_LambdaBack.tail<2>() * (nodeData.m_contactParameter.m_params[CMT::d_TIdx] / m_settings.m_deltaT);
            }

            lambda_T = - (nodeData.m_R_i_inv_diag.tail<2>().asDiagonal() * lambda_T).eval();
            lambda_T += nodeData.m_LambdaBack.tail<2>();

            //Prox
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
            Prox::ProxFunction<ConvexSets::Disk>::doProxSingle( nodeData.m_contactParameter.m_params[CMT::muIdx] * nodeData.m_LambdaFront(0), lambda_T );

            nodeData.m_LambdaFront.tail<2>() =  lambda_T;

            lambda_T = lambda_T - nodeData.m_LambdaBack.tail<2>(); // Delta lambda_T


            if( nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u1BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1.rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);

                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_settings.m_AbsTol, m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(      uCache1,
                                                 nodeData.m_pCollData->m_pBody1->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
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
            if( nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                nodeData.m_u2BufferPtr->m_front +=
                    nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal()*nodeData.m_W_body2.rightCols<2>() * lambda_T;


                LOGSLLEVEL3_CONTACT(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);


                if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache2,
                                                 nodeData.m_u2BufferPtr->m_front,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                        }
                    } else {
                        m_bConverged=false;
                    }

                } else if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix) {
                    if(m_globalIterationCounter >= m_settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaMatrixNorm(   uCache2,
                                                 nodeData.m_pCollData->m_pBody2->m_MassMatrix_diag,
                                                 nodeData.m_LambdaBack,
                                                 nodeData.m_LambdaFront,
                                                 nodeData.m_G_ii,
                                                 m_settings.m_AbsTol,
                                                 m_settings.m_RelTol);
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
            // =========================================================



            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_settings.m_AbsTol, m_settings.m_RelTol)) {
                LOGSLLEVEL3_CONTACT(m_pSolverLog , "\t---> Lambda converged"<<std::endl);
            }




            if(m_settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_settings.m_MinIter && (m_bConverged || m_settings.m_bComputeResidual) ) {
                    nodeData.m_bConverged = Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,
                                            nodeData.m_LambdaFront,
                                            m_settings.m_AbsTol,
                                            m_settings.m_RelTol,
                                            residual
                                                                         );
                    m_pGraph->m_maxResidual = std::max(residual,m_pGraph->m_maxResidual);
                    if(!nodeData.m_bConverged) {
                        // Set global Converged = false;
                        m_bConverged = false;
                    }
                } else {
                    m_bConverged=false;
                }
            }


            // Swap Lambdas, but dont swap Velocities...
            //nodeData.m_LambdaBack = nodeData.m_LambdaFront; // necessary if we use doVelocityUpdate function!
            nodeData.swapLambdas(); // faster only switch pointers



        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }
    }

};

/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
class SorProxInitNodeVisitor {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = ContactGraphIteration;
    using NodeDataType = typename ContactGraphType::NodeDataType;
    using EdgeDataType = typename ContactGraphType::EdgeDataType;
    using EdgeType = typename ContactGraphType::EdgeType;
    using NodeType = typename ContactGraphType::NodeType;

    SorProxInitNodeVisitor(const InclusionSolverSettingsType &settings): m_alpha(1), m_settings(settings)
    {}

    void setLog(Logging::Log * solverLog) {
        m_pSolverLog = solverLog;
    }

    // Set Sor Prox parameter, before calling visitNode
    void setParams(PREC alpha) {
        m_alpha = alpha;
    }


    template<int bodyNr>
    inline void computeW_UCF(NodeDataType & nodeData) {

            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData * pCollData;

            pCollData = nodeData.m_pCollData;

            if(bodyNr == 1) {
                //Set matrix size!
                //nodeData.m_W_body1.setZero());

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
                //nodeData.m_W_body2.setZero(NDOFuBody, ContactModels::getLambdaDim(ContactModels::Enum::UCF));

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
    }


    void visitNode(NodeType & node) {
        auto & nodeData = node.m_nodeData;

        // Initialize for UCF Contact models
        if( nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCF ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD ||
            nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD  ) {

            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            const unsigned int dimSet = ContactModels::getLambdaDim(ContactModels::Enum::UCF);

            // chi and eps was already initialized in contactGraph!
            //nodeData.m_b.setZero(dimSet);
            nodeData.m_LambdaBack.setZero();
            nodeData.m_LambdaFront.setZero();
            //nodeData.m_R_i_inv_diag.setZero(dimSet);
            //nodeData.m_G_ii.setZero(dimSet,dimSet);

            // =========================================================================================================

            // Compute generalized force directions W
            auto state = nodeData.m_pCollData->m_pBody1->m_eMode;
            if(  state == RigidBodyType::BodyMode::SIMULATED){
                computeW_UCF<1>(nodeData);
            }else if(state == RigidBodyType::BodyMode::ANIMATED){
                // Contact goes into xi_N, xi_T
                ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
            }
            state = nodeData.m_pCollData->m_pBody2->m_eMode;
            if(  state == RigidBodyType::BodyMode::SIMULATED){
                computeW_UCF<2>(nodeData);
            }else if(state == RigidBodyType::BodyMode::ANIMATED){
                // Contact goes into xi_N, xi_T
                ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
            }


            // Get lambda from percussion pool otherwise set to zero
            // TODO
            nodeData.m_LambdaBack.setZero();

            // (1+e)*xi -> b
            nodeData.m_b = (nodeData.m_eps.array() + 1).matrix().asDiagonal() * nodeData.m_chi;

            // u_0 , calculate const b
            // First Body
            if(nodeData.m_pCollData->m_pBody1->m_eMode == RigidBodyType::BodyMode::SIMULATED) {

                // m_front is zero here-> see DynamicsSystem sets it to zero!
                nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack );
                /// + initial values M^⁻1 W lambda0 from percussion pool

                nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
                nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
            }
            // SECOND BODY!
            if(nodeData.m_pCollData->m_pBody2->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {

                // m_front is zero here-> see DynamicsSystem sets it to zero!
                nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack );
                /// + initial values M^⁻1 W lambda0 from percussion pool

                nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u2BufferPtr->m_back;
                nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
            }

            if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFD);
                Vector3 d(  nodeData.m_contactParameter.m_params[CMT::d_NIdx], //d_N
                            nodeData.m_contactParameter.m_params[CMT::d_TIdx], //d_T
                            nodeData.m_contactParameter.m_params[CMT::d_TIdx]);//d_T
                nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*d;
            } else if(nodeData.m_contactParameter.m_contactModel == ContactModels::Enum::UCFDD) {
                using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCFDD);
                Vector3 d(  nodeData.m_contactParameter.m_params[CMT::d_NIdx],    //d_N
                            nodeData.m_contactParameter.m_params[CMT::d_TfixIdx], //d_TFix
                            nodeData.m_contactParameter.m_params[CMT::d_TfixIdx]);//d_TFix
                nodeData.m_G_ii.diagonal() += 1.0/m_settings.m_deltaT*d;
            }

            // Calculate R_ii

            if(m_settings.m_eMethod == InclusionSolverSettingsType::SOR_CONTACT_DS){
                // De Saxe Formulation, only one r parameter because friction cone
                if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_SUM){
                   nodeData.m_R_i_inv_diag.setConstant( m_alpha / nodeData.m_G_ii.diagonal().sum() );
                }
                else if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_SUM2){
                   nodeData.m_R_i_inv_diag.setConstant( m_alpha / (nodeData.m_G_ii.diagonal().sum() *3) );
                }
                else if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_MAX){
                   nodeData.m_R_i_inv_diag.setConstant( m_alpha / nodeData.m_G_ii.diagonal().maxCoeff() );
                }else{
                    ERRORMSG(" You specified a R-Matrix strategy which has not been implemented so far!");
                }

            }else{
                // Alart Curnier Formulation
                // De Saxe Formulation, only one r parameter because friction cone
                if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_SUM){

                   nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);
                   nodeData.m_R_i_inv_diag.tail<2>().setConstant( m_alpha / nodeData.m_G_ii.diagonal().tail<2>().sum() );

                }else if(m_settings.m_RStrategy == InclusionSolverSettingsType::RSTRATEGY_MAX){
                    // Take also offdiagonal values for lambda_N
                    //nodeData.m_R_i_inv_diag(0) = m_alpha / std::max(std::max(nodeData.m_G_ii(0,0), nodeData.m_mu(0)*nodeData.m_G_ii(0,1)), nodeData.m_mu(0)*nodeData.m_G_ii(0,2));
                    // Take only diagonal
                    nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);
                    PREC r_T = m_alpha / (nodeData.m_G_ii.diagonal().tail<2>().maxCoeff());
                    nodeData.m_R_i_inv_diag(1) = r_T;
                    nodeData.m_R_i_inv_diag(2) = r_T;
                }else{
                    ERRORMSG(" You specified a R-Matrix strategy which has not been implemented so far!");
                }


            }



            LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t ---> nd.m_b: "<< nodeData.m_b.transpose() <<std::endl
                << "\t ---> nd.m_G_ii: "<<std::endl<< nodeData.m_G_ii <<std::endl
                << "\t ---> nd.m_R_i_inv_diag: "<< nodeData.m_R_i_inv_diag.transpose() <<std::endl;);



            LOGSLLEVEL3_CONTACT(m_pSolverLog,  "\t ---> nd.m_mu: "<< nodeData.m_contactParameter.m_params[CMT::muIdx] <<std::endl;);

        } else {
            ERRORMSG(" You specified a contact model which has not been implemented so far!");
        }

    }

private:
    Logging::Log * m_pSolverLog;
    PREC m_alpha;
    InclusionSolverSettingsType m_settings;
};





#endif

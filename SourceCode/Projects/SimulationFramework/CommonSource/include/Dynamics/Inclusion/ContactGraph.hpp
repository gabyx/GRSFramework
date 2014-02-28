#ifndef ContactGraph_hpp
#define ContactGraph_hpp


/** Contact Graph */
#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"


#include "GeneralGraph.hpp"
#include "CollisionData.hpp"
#include "ContactModels.hpp"
#include "ContactParameterMap.hpp"
#include "ContactGraphNodeData.hpp"

#include InclusionSolverSettings_INCLUDE_FILE

#include "VectorToSkewMatrix.hpp"
#include "ProxFunctions.hpp"

template<typename ContactGraphMode > class ContactGraph;

struct ContactGraphMode{
    struct NoIteration{};
    struct ForIteration{};
};

template <>
class ContactGraph<ContactGraphMode::NoIteration> : public Graph::GeneralGraph< ContactGraphNodeData,ContactGraphEdgeData > {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef ContactGraphNodeData NodeDataType;
    typedef ContactGraphEdgeData EdgeDataType;
    typedef typename Graph::Edge< NodeDataType, EdgeDataType> EdgeType;
    typedef typename Graph::Node< NodeDataType, EdgeDataType> NodeType;

    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType NodeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType EdgeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType NodeListIteratorType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType EdgeListIteratorType;

public:

    ContactGraph(ContactParameterMap * contactParameterMap):
    m_nodeCounter(0),m_edgeCounter(0), m_nLambdas(0),m_nFrictionParams(0)
    {
        m_pContactParameterMap = contactParameterMap;
    }
    ~ContactGraph() {
        clearGraph();
    }

    void clearGraph() {
        // This deletes all nodes, edges, and decrements the reference counts for the nodedata and edgedata
        // cleanup allocated memory
        for(NodeListIteratorType n_it = this->m_nodes.begin(); n_it != this->m_nodes.end(); n_it++)
            delete (*n_it);
        for(EdgeListIteratorType e_it = this->m_edges.begin(); e_it != this->m_edges.end(); e_it++)
            delete (*e_it);
        //cout << "clear graph"<<endl;
        this->m_nodes.clear();
        m_nodeCounter = 0;
        this->m_edges.clear();
        m_edgeCounter = 0;
        m_nLambdas =0;
        m_nFrictionParams=0;
        m_simBodiesToContactsList.clear();

    }

    void addNode(CollisionData * pCollData) {

        ASSERTMSG(pCollData->m_pBody1 != NULL && pCollData->m_pBody2 != NULL, " Bodys are null pointers?");
        //cout << "add node : "<<m_nodeCounter<< " body id:" << pCollData->m_pBody1->m_id <<" and "<< pCollData->m_pBody2->m_id <<endl;

        //  add a contact node to the graph
        // check to which nodes we need to connect?
        // all nodes (contacts) which are in the BodyContactList (maps bodies -> contacts)

        // add the pNodeData to the node list
        this->m_nodes.push_back( new NodeType(m_nodeCounter));
        NodeType * addedNode = this->m_nodes.back();
        addedNode->m_nodeData.m_pCollData = pCollData;



        // Specify the contact model, (here we should do a look up or what ever)! ==================================
        addedNode->m_nodeData.m_eContactModel = ContactModels::NCF_ContactModel;
        addedNode->m_nodeData.m_nLambdas = ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension;
        addedNode->m_nodeData.m_chi.setZero(addedNode->m_nodeData.m_nLambdas);
        // =========================================================================================================


        // Compute general parameters for the contact
        computeParams(addedNode->m_nodeData);

        // FIRST BODY!
        if( pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {

            computeW<1>( addedNode->m_nodeData);
            connectNode<1>( addedNode);

        } else if( pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }


        // SECOND BODY!
        if( pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {

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


    static const Eigen::Matrix<PREC,NDOFuBody,Eigen::Dynamic> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const Eigen::Matrix<PREC,NDOFuBody,Eigen::Dynamic> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::unordered_map<const RigidBodyType *, NodeListType > m_simBodiesToContactsList;
    typedef typename std::unordered_map<const RigidBodyType *, NodeListType >::iterator  BodyToContactsListIterator;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.
private:

    void computeParams(NodeDataType & nodeData) {
        if( nodeData.m_eContactModel == ContactModels::NCF_ContactModel ) {
            // Get Contact Parameters

            //Set matrix size!
            nodeData.m_I_plus_eps.setZero(ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);
            nodeData.m_mu.setZero(ContactModels::NormalAndCoulombFrictionContactModel::nFrictionParams);

            ContactParams & params  = m_pContactParameterMap->getContactParams(nodeData.m_pCollData->m_pBody1->m_eMaterial,nodeData.m_pCollData->m_pBody2->m_eMaterial);
            nodeData.m_mu(0)         = params.m_mu;
            nodeData.m_I_plus_eps(0)    = 1 + params.m_epsilon_N;
            nodeData.m_I_plus_eps(1)    = 1 + params.m_epsilon_T;
            nodeData.m_I_plus_eps(2)    = 1 + params.m_epsilon_T;
        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }

    }

    template<int bodyNr>
    void computeW(NodeDataType & nodeData) {



        if(nodeData.m_eContactModel == ContactModels::NCF_ContactModel) {


            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData * pCollData;

            pCollData = nodeData.m_pCollData;



            if(bodyNr == 1) {
                nodeData.m_W_body1.setZero(NDOFuBody, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

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
                nodeData.m_W_body2.setZero(NDOFuBody, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

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
        NodeListType & nodeList = m_simBodiesToContactsList[pBody];
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
            //cout << "add edge: "<<pNode->m_nodeNumber<<" to "<<(*it)->m_nodeNumber<<" body Id:"<< pBody->m_id<<endl;
        }

        // Add new Node to the list;
        nodeList.push_back(pNode);

    }

    ContactParameterMap* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter; ///< An edge counter, starting at 0.

};

template <>
class ContactGraph<ContactGraphMode::ForIteration> : public Graph::GeneralGraph< ContactGraphNodeDataIteration,ContactGraphEdgeData > {
public:


    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef ContactGraphNodeDataIteration NodeDataType;
    typedef ContactGraphEdgeData EdgeDataType;
    typedef typename Graph::Node< NodeDataType, EdgeDataType> NodeType;
    typedef typename Graph::Edge< NodeDataType, EdgeDataType> EdgeType;

    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType NodeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType EdgeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType NodeListIteratorType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType EdgeListIteratorType;

    ContactGraph(ContactParameterMap * contactParameterMap):
    m_nodeCounter(0),m_edgeCounter(0), m_nLambdas(0),m_nFrictionParams(0)
    {
        m_pContactParameterMap = contactParameterMap;
    }

    ~ContactGraph() {
        clearGraph();
    }

    void clearGraph() {
        // This deletes all nodes, edges, and decrements the reference counts for the nodedata and edgedata
        // cleanup allocated memory
        for(NodeListIteratorType n_it = this->m_nodes.begin(); n_it != this->m_nodes.end(); n_it++)
            delete (*n_it);
        for(EdgeListIteratorType e_it = this->m_edges.begin(); e_it != this->m_edges.end(); e_it++)
            delete (*e_it);
        //cout << "clear graph"<<endl;
        this->m_nodes.clear();
        m_nodeCounter = 0;
        this->m_edges.clear();
        m_edgeCounter = 0;
        m_nLambdas =0;
        m_nFrictionParams=0;
        m_simBodiesToContactsList.clear();

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


    static const Eigen::Matrix<PREC,NDOFuBody,Eigen::Dynamic> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const Eigen::Matrix<PREC,NDOFuBody,Eigen::Dynamic> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::unordered_map<const RigidBodyType *, NodeListType > m_simBodiesToContactsList;
    typedef typename std::unordered_map<const RigidBodyType *, NodeListType >::iterator  BodyToContactsListIteratorType;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.
private:

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
                nodeData.m_W_body1.setZero(NDOFuBody, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

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
                nodeData.m_W_body2.setZero(NDOFuBody, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

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
        NodeListType & nodeList = m_simBodiesToContactsList[pBody];
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
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
class SorProxStepNodeVisitor{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    typedef ContactGraph<ContactGraphMode::ForIteration> ContactGraphType;
    typedef typename ContactGraphType::NodeDataType NodeDataType;
    typedef typename ContactGraphType::EdgeDataType EdgeDataType;
    typedef typename ContactGraphType::EdgeType EdgeType;
    typedef typename ContactGraphType::NodeType NodeType;


    SorProxStepNodeVisitor(const InclusionSolverSettingsType &settings,
                           bool & globalConverged, const unsigned int & globalIterationNeeded):
            m_Settings(settings),m_bConverged(globalConverged),
            m_globalIterationCounter(globalIterationNeeded)
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
            LOG(m_pSolverLog, "---> SorProx, Node: " << node.m_nodeNumber <<"====================="<<  std::endl);
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED  &&  nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED){
               LOG(m_pSolverLog, "\t---> Sim<->Sim Node:"<<  std::endl);
            }
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

            // Experimental
            //Relaxation term damper (take care R_i_inv needs to be initialized as well!)
            //nodeData.m_LambdaFront += nodeData.m_LambdaBack * m_Settings.m_dinv;

            nodeData.m_LambdaFront = -(nodeData.m_R_i_inv_diag.asDiagonal() * nodeData.m_LambdaFront).eval(); //No alias due to diagonal!!! (if normal matrix multiplication there is aliasing!
            nodeData.m_LambdaFront += nodeData.m_LambdaBack;




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

            Prox::ProxFunction<ConvexSets::RPlusAndDisk>::doProxSingle(
                nodeData.m_mu(0),
                nodeData.m_LambdaFront.head<ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension>()
            );

#if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "\t---> nd.m_LambdaBack: "  << nodeData.m_LambdaBack.transpose() << std::endl);
            LOG(m_pSolverLog, "\t---> nd.m_LambdaFront: " << nodeData.m_LambdaFront.transpose() << std::endl);
            if(Numerics::cancelCriteriaValue(nodeData.m_LambdaBack,nodeData.m_LambdaFront,m_Settings.m_AbsTol, m_Settings.m_RelTol)){
              *m_pSolverLog <<"\t---> Lambda converged"<<std::endl;
            }
#endif

            // u_k+1 = u_k + M^-1 W (lambda_k+1 - lambda_k)
            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED ) {
                uCache1 = nodeData.m_u1BufferPtr->m_front;
                nodeData.m_u1BufferPtr->m_front = nodeData.m_u1BufferPtr->m_front + nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 * ( nodeData.m_LambdaFront - nodeData.m_LambdaBack );

               #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"\t---> nd.u1Front: " << nodeData.m_u1BufferPtr->m_front.transpose() << std::endl);
               #endif


                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
                        nodeData.m_bConverged  = Numerics::cancelCriteriaValue(uCache1,nodeData.m_u1BufferPtr->m_front,m_Settings.m_AbsTol, m_Settings.m_RelTol);
                        if(!nodeData.m_bConverged ) {
                            //converged stays false;
                            // Set global Converged = false;
                            m_bConverged = false;
                            *m_pSolverLog << "\t---> m_bConverged = false;"<<std::endl;
                        }
                    } else {
                        m_bConverged=false;
                    }
                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix){
                    if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
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

                #if CoutLevelSolverWhenContact>2
                LOG(m_pSolverLog,"\t---> nd.u2Front: " << nodeData.m_u2BufferPtr->m_front.transpose() << std::endl);
                #endif

                if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InVelocityLocal) {
                    if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
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

                }else if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InEnergyLocalMix){
                    if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
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

            if(m_Settings.m_eConvergenceMethod == InclusionSolverSettingsType::InLambda) {
                if(m_globalIterationCounter >= m_Settings.m_MinIter && m_bConverged) {
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
            nodeData.swapLambdas();


        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }
    }

private:
    Logging::Log * m_pSolverLog;
    const InclusionSolverSettingsType & m_Settings;
    bool & m_bConverged; ///< Access to global flag for cancelation criteria
    const unsigned int & m_globalIterationCounter; ///< Access to global iteration counter

};


/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
class SorProxInitNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef ContactGraph<ContactGraphMode::ForIteration> ContactGraphType;
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
        nodeData.m_b = nodeData.m_I_plus_eps.asDiagonal() * nodeData.m_chi;

        // u_0 , calculate const b
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::BodyState::SIMULATED) {

            // m_front is zero here-> see DynamicsSystem sets it to zero!
            nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack );
            /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::BodyState::SIMULATED ) {

            // m_front is zero here-> see DynamicsSystem sets it to zero!
            nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack );
            /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u2BufferPtr->m_back;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }



        // Calculate R_ii

        // Take also offdiagonal values for lambda_N
        //nodeData.m_R_i_inv_diag(0) = m_alpha / std::max(std::max(nodeData.m_G_ii(0,0), nodeData.m_mu(0)*nodeData.m_G_ii(0,1)), nodeData.m_mu(0)*nodeData.m_G_ii(0,2));
        // Take only diagonal
        nodeData.m_R_i_inv_diag(0) = m_alpha / nodeData.m_G_ii(0,0);

        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().tail<2>()).maxCoeff());
        nodeData.m_R_i_inv_diag(1) = r_T;
        nodeData.m_R_i_inv_diag(2) = r_T;

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog, "\t ---> nd.m_b: "<< nodeData.m_b.transpose() <<std::endl
                << "\t ---> nd.m_G_ii: "<<std::endl<< nodeData.m_G_ii <<std::endl
                << "\t ---> nd.m_R_i_inv_diag: "<< nodeData.m_R_i_inv_diag.transpose() <<std::endl;);
        #endif

        #if CoutLevelSolverWhenContact>2
            LOG(m_pSolverLog,  "\t ---> nd.m_mu: "<< nodeData.m_mu <<std::endl;);
        #endif


    }

private:
    Logging::Log * m_pSolverLog;
    PREC m_alpha;
};




#endif

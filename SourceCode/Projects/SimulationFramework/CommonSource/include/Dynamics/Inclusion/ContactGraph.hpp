#ifndef ContactGraph_hpp
#define ContactGraph_hpp


/** Contact Graph */
#include <TypeDefs.hpp>

#include "AssertionDebug.hpp"

#include "GeneralGraph.hpp"

#include "ContactModels.hpp"
#include "ContactParameterMap.hpp"
#include "CollisionData.hpp"



template<typename TRigidBody>
class ContactGraphNodeData {
public:

    typedef TRigidBody RigidBodyType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeData(): m_pCollData(NULL) {
        m_W_body1.setZero();
        m_W_body2.setZero();
        m_xi.setZero();
        m_mu.setZero();
        m_I_plus_eps.setZero();
        m_eps.setZero();
        m_nLambdas = 0;
    }

    ContactGraphNodeData(CollisionData<RigidBodyType> * collDataPtr): m_pCollData(collDataPtr) {}

    Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> m_W_body1;
    Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> m_W_body2;
    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_xi;

    Eigen::Matrix<PREC,Eigen::Dynamic,1>  m_I_plus_eps;
    Eigen::Matrix<PREC,Eigen::Dynamic,1>  m_eps;
    Eigen::Matrix<PREC,Eigen::Dynamic,1>  m_mu;

    unsigned int m_nLambdas;

    const CollisionData<RigidBodyType> * m_pCollData;

    ContactModels::ContactModelEnum m_eContactModel;                  ///< This is a generic type which is used to distinguish between the different models!. See namespace ContactModels.
};

template<typename TRigidBody>
class ContactGraphNodeDataIteration : public ContactGraphNodeData<TRigidBody> {
public:

    typedef TRigidBody RigidBodyType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataIteration()
    {
        m_LambdaBack.setZero();
        m_LambdaFront.setZero();

        m_b.setZero();

        m_u1BufferPtr = NULL;
        m_u2BufferPtr = NULL;

        m_bConverged = 0;
    }


    ~ContactGraphNodeDataIteration(){
    }

    FrontBackBuffer<VectorUObj,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> * m_u1BufferPtr; ///< Pointers into the right Front BackBuffer for bodies 1 and 2
    FrontBackBuffer<VectorUObj,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> * m_u2BufferPtr; /// Only valid for Simulated Objects


    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_LambdaBack;
    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_LambdaFront;

    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_R_i_inv_diag; // Build over G_ii
    Eigen::Matrix<PREC,Eigen::Dynamic,Eigen::Dynamic> m_G_ii; // just for R_ii, and maybee later for better solvers!

    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_b;

    bool m_bConverged;

    void swapVelocities() {

        if(m_u1BufferPtr){ m_u1BufferPtr->m_Back.swap(m_u1BufferPtr->m_Front); }

        if(m_u2BufferPtr){m_u2BufferPtr->m_Back.swap(m_u2BufferPtr->m_Front); }
    };

    void swapLambdas() {
        m_LambdaBack.swap(m_LambdaFront);
    };

};

/*
* The EdgeData class for the Contact Graph, nothing is deleted in this class, this is plain old data!
*/
template<typename TRigidBody>
class ContactGraphEdgeData {
public:

    typedef TRigidBody RigidBodyType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphEdgeData(): m_pBody(NULL) {};

    RigidBodyType * m_pBody; // Tells to which body this edges belongs!

    //Eigen::Matrix<PREC,NDOFFriction+1,NDOFFriction+1> m_G_SE; // Start Node 1 to End Node 3 = G_13

};

template<typename TRigidBody, typename ContactGraphMode > class ContactGraph;

struct ContactGraphMode{
    struct NoIteration{};
    struct ForIteration{};
};

template < typename TRigidBody>
class ContactGraph<TRigidBody, ContactGraphMode::NoIteration> : public Graph::GeneralGraph< ContactGraphNodeData<TRigidBody>,ContactGraphEdgeData<TRigidBody> > {
public:


    typedef TRigidBody RigidBodyType;
    typedef typename RigidBodyType::LayoutConfigType LayoutConfigType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType);

    typedef ContactGraphNodeData<RigidBodyType> NodeDataType;
    typedef ContactGraphEdgeData<RigidBodyType> EdgeDataType;
    typedef typename Graph::Edge< NodeDataType, EdgeDataType> EdgeType;
    typedef typename Graph::Node< NodeDataType, EdgeDataType> NodeType;

    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType NodeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType EdgeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType NodeListIteratorType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType EdgeListIteratorType;

public:

    ContactGraph(const ContactParameterMap<RigidBodyType> * contactParameterMap):
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
        m_SimBodyToContactsList.clear();

    }

    void addNode(CollisionData<RigidBodyType> * pCollData) {

        ASSERTMSG(pCollData->m_pBody1.get() != NULL && pCollData->m_pBody2.get() != NULL, " Bodys are null pointers?");
        //cout << "add node : "<<m_nodeCounter<< " body id:" << pCollData->m_pBody1->m_id <<" and "<< pCollData->m_pBody2->m_id <<endl;

        //  add a contact node to the graph
        // check to which nodes we need to connect?
        // all nodes (contacts) which are in the BodyContactList (maps bodies -> contacts)

        // add the pNodeData to the node list
        this->m_nodes.push_back( new NodeType(m_nodeCounter));
        NodeType * addedNode = this->m_nodes.back();
        addedNode->m_nodeData.m_pCollData = pCollData;



        // Specify the contact model, (here we should do a look up or what ever)! ==================================
        addedNode->m_nodeData.m_eContactModel = ContactModels::NCFContactModel;
        addedNode->m_nodeData.m_nLambdas = ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension;
        addedNode->m_nodeData.m_xi.setZero(addedNode->m_nodeData.m_nLambdas);
        // =========================================================================================================


        // Compute general parameters for the contact
        computeParams(addedNode->m_nodeData);

        // FIRST BODY!
        if( pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {

            computeW<1>( addedNode->m_nodeData);
            connectNode<1>( addedNode);

        } else if( pCollData->m_pBody1->m_eState == RigidBodyType::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }


        // SECOND BODY!
        if( pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {

            computeW<2>( addedNode->m_nodeData);
            connectNode<2>( addedNode);

        } else if( pCollData->m_pBody2->m_eState == RigidBodyType::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }


        // increment the lambda counter, of how many forces we have so far!
        m_nLambdas += addedNode->m_nodeData.m_nLambdas;
        m_nFrictionParams += addedNode->m_nodeData.m_mu.rows();

        m_nodeCounter++;
    }


    static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1.get()  == pBody || nodeData.m_pCollData->m_pBody2.get()  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1.get() == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1.get() == pBody || nodeData.m_pCollData->m_pBody2.get()  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1.get() == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::map<const RigidBodyType *, NodeListType > m_SimBodyToContactsList;
    typedef typename std::map<const RigidBodyType *, NodeListType >::iterator  BodyToContactsListIterator;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.
private:

    void computeParams(NodeDataType & nodeData) {
        if( nodeData.m_eContactModel == ContactModels::NCFContactModel ) {
            // Get Contact Parameters

            //Set matrix size!
            nodeData.m_I_plus_eps.setZero(ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);
            nodeData.m_mu.setZero(ContactModels::NormalAndCoulombFrictionContactModel::nFrictionParams);

            ContactParams<LayoutConfigType> & params  = m_pContactParameterMap->getContactParams(nodeData.m_pCollData->m_pBody1->m_eMaterial,nodeData.m_pCollData->m_pBody2->m_eMaterial);
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



        if(nodeData.m_eContactModel == ContactModels::NCFContactModel) {


            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData<RigidBodyType> * pCollData;

            pCollData = nodeData.m_pCollData;



            if(bodyNr == 1) {
                nodeData.m_W_body1.setZero(NDOFuObj, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

                updateSkewSymmetricMatrix<>( pCollData->m_r_S1C1, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody1->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body1.col(0).template head<3>() = - pCollData->m_e_z; // I frame
                nodeData.m_W_body1.col(0).template tail<3>() = - I_Jacobi_2 * pCollData->m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body1.col(1).template head<3>() = - pCollData->m_e_x; // I frame
                nodeData.m_W_body1.col(1).template tail<3>() = - I_Jacobi_2 * pCollData->m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body1.col(2).template head<3>() = - pCollData->m_e_y; // I frame
                nodeData.m_W_body1.col(2).template tail<3>() = - I_Jacobi_2 * pCollData->m_e_y;
            } else {
                nodeData.m_W_body2.setZero(NDOFuObj, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

                updateSkewSymmetricMatrix<>( pCollData->m_r_S2C2, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody2->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body2.col(0).template head<3>() =  pCollData->m_e_z; // I frame
                nodeData.m_W_body2.col(0).template tail<3>() =  I_Jacobi_2 * pCollData->m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body2.col(1).template head<3>() =  pCollData->m_e_x; // I frame
                nodeData.m_W_body2.col(1).template tail<3>() =  I_Jacobi_2 * pCollData->m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body2.col(2).template head<3>() =  pCollData->m_e_y; // I frame
                nodeData.m_W_body2.col(2).template tail<3>() =  I_Jacobi_2 * pCollData->m_e_y;
            }
        } else {
            ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }

    }

    template<int bodyNr>
    void connectNode(NodeType * pNode) {

        EdgeType * addedEdge;
        RigidBodyType * pBody = (bodyNr==1)? pNode->m_nodeData.m_pCollData->m_pBody1.get() : pNode->m_nodeData.m_pCollData->m_pBody2.get();

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
            //cout << "add edge: "<<pNode->m_nodeNumber<<" to "<<(*it)->m_nodeNumber<<" body Id:"<< pBody->m_id<<endl;
        }

        // Add new Node to the list;
        nodeList.push_back(pNode);

    }

    ContactParameterMap<RigidBodyType>* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter; ///< An edge counter, starting at 0.

};

template < typename TRigidBody>
class ContactGraph<TRigidBody,ContactGraphMode::ForIteration> : public Graph::GeneralGraph< ContactGraphNodeDataIteration<TRigidBody>,ContactGraphEdgeData<TRigidBody> > {
public:


    typedef TRigidBody RigidBodyType;
    typedef typename RigidBodyType::LayoutConfigType LayoutConfigType;
    DEFINE_LAYOUT_CONFIG_TYPES_OF(RigidBodyType::LayoutConfigType);

    typedef ContactGraphNodeDataIteration<RigidBodyType> NodeDataType;
    typedef ContactGraphEdgeData<RigidBodyType> EdgeDataType;
    typedef typename Graph::Node< NodeDataType, EdgeDataType> NodeType;
    typedef typename Graph::Edge< NodeDataType, EdgeDataType> EdgeType;

    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType NodeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType EdgeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType NodeListIteratorType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType EdgeListIteratorType;

    typedef typename Graph::NodeVisitor<NodeDataType,EdgeDataType> NodeVisitorType;

    ContactGraph(ContactParameterMap<RigidBodyType> * contactParameterMap):
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
        m_SimBodyToContactsList.clear();

    }

    void addNode(CollisionData<RigidBodyType> * pCollData) {

        //Take care state, is only q = q_m, u is not set and is zero!

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
        // TODO
        addedNode->m_nodeData.m_eContactModel = ContactModels::NCFContactModel;
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
        if( pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {

            //Set Flag that this Body in ContactGraph
            pCollData->m_pBody1->m_pSolverData->m_bInContactGraph = true;
            // Unset the flag when this Node is removed;

            //Link to FrontBackBuffer
            addedNode->m_nodeData.m_u1BufferPtr = & pCollData->m_pBody1->m_pSolverData->m_uBuffer;

            computeW<1>( addedNode->m_nodeData);
            connectNode<1>( addedNode);

        } else if( pCollData->m_pBody1->m_eState == RigidBodyType::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }


        // SECOND BODY!
        if( pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {

            //Set Flag that this Body in ContactGraph
            pCollData->m_pBody2->m_pSolverData->m_bInContactGraph = true;
            // Unset the flag when this Node is removed;

            //Link to FrontBackBuffer
            addedNode->m_nodeData.m_u2BufferPtr = & pCollData->m_pBody2->m_pSolverData->m_uBuffer;

            computeW<2>( addedNode->m_nodeData);
            connectNode<2>( addedNode);

        } else if( pCollData->m_pBody2->m_eState == RigidBodyType::ANIMATED ) {
            // Contact goes into xi_N, xi_T
            ASSERTMSG(false,"RigidBody<TLayoutConfig>::ANIMATED objects have not been implemented correctly so far!");
        }

        // increment the lambda counter, of how many forces we have so far!
        m_nLambdas += addedNode->m_nodeData.m_nLambdas;
        m_nFrictionParams += addedNode->m_nodeData.m_mu.rows();

        m_nodeCounter++;
    }


    static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1.get()  == pBody || nodeData.m_pCollData->m_pBody2.get()  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1.get() == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1.get() == pBody || nodeData.m_pCollData->m_pBody2.get()  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1.get() == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::map<const RigidBodyType *, NodeListType > m_SimBodyToContactsList;
    typedef typename std::map<const RigidBodyType *, NodeListType >::iterator  BodyToContactsListIteratorType;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.
private:

    void computeParams(NodeDataType & nodeData) {
        if( nodeData.m_eContactModel == ContactModels::NCFContactModel ) {
            // Get Contact Parameters


            //Set matrix size!
            nodeData.m_I_plus_eps.setZero(ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);
            nodeData.m_eps.setZero(ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);
            nodeData.m_mu.setZero(ContactModels::NormalAndCoulombFrictionContactModel::nFrictionParams);



            ContactParams<LayoutConfigType> & params  = m_pContactParameterMap->getContactParams(nodeData.m_pCollData->m_pBody1->m_eMaterial,nodeData.m_pCollData->m_pBody2->m_eMaterial);
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



        if(nodeData.m_eContactModel == ContactModels::NCFContactModel) {


            static Matrix33 I_r_SiCi_hat = Matrix33::Zero();
            static Matrix33 I_Jacobi_2; // this is the second part of the Jacobi;
            static const CollisionData<RigidBodyType> * pCollData;

            pCollData = nodeData.m_pCollData;



            if(bodyNr == 1) {
                //Set matrix size!
                nodeData.m_W_body1.setZero(NDOFuObj, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

                updateSkewSymmetricMatrix<>( pCollData->m_r_S1C1, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody1->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body1.col(0).template head<3>() = - pCollData->m_e_z; // I frame
                nodeData.m_W_body1.col(0).template tail<3>() = - I_Jacobi_2 * pCollData->m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body1.col(1).template head<3>() = - pCollData->m_e_x; // I frame
                nodeData.m_W_body1.col(1).template tail<3>() = - I_Jacobi_2 * pCollData->m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body1.col(2).template head<3>() = - pCollData->m_e_y; // I frame
                nodeData.m_W_body1.col(2).template tail<3>() = - I_Jacobi_2 * pCollData->m_e_y;
            } else {
                //Set matrix size!
                nodeData.m_W_body2.setZero(NDOFuObj, ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension);

                updateSkewSymmetricMatrix<>( pCollData->m_r_S2C2, I_r_SiCi_hat);
                I_Jacobi_2 = ( nodeData.m_pCollData->m_pBody2->m_A_IK.transpose() * I_r_SiCi_hat );
                // N direction =================================================
                nodeData.m_W_body2.col(0).template head<3>() =  pCollData->m_e_z; // I frame
                nodeData.m_W_body2.col(0).template tail<3>() =  I_Jacobi_2 * pCollData->m_e_z;

                // T1 direction =================================================
                nodeData.m_W_body2.col(1).template head<3>() =  pCollData->m_e_x; // I frame
                nodeData.m_W_body2.col(1).template tail<3>() =  I_Jacobi_2 * pCollData->m_e_x;

                // T2 direction =================================================
                nodeData.m_W_body2.col(2).template head<3>() =  pCollData->m_e_y; // I frame
                nodeData.m_W_body2.col(2).template tail<3>() =  I_Jacobi_2 * pCollData->m_e_y;
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
            //cout << "add edge: "<<pNode->m_nodeNumber<<" to "<<(*it)->m_nodeNumber<<" body Id:"<< pBody->m_id<<endl;
        }

        // Add new Node to the list;
        nodeList.push_back(pNode);

    }

    ContactParameterMap<RigidBodyType>* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter; ///< An edge counter, starting at 0.

};






#endif

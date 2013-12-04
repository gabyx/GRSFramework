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

#include "CompileTimeArray.hpp"


class ContactGraphNodeData {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
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

    ContactGraphNodeData(CollisionData * collDataPtr): m_pCollData(collDataPtr) {}

    Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> m_W_body1;
    Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> m_W_body2;
    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_xi;

    Eigen::Matrix<PREC,Eigen::Dynamic,1>  m_I_plus_eps;
    Eigen::Matrix<PREC,Eigen::Dynamic,1>  m_eps;
    Eigen::Matrix<PREC,Eigen::Dynamic,1>  m_mu;

    unsigned int m_nLambdas;

    unsigned int m_nodeColor;

    const CollisionData * m_pCollData;

    ContactModels::ContactModelEnum m_eContactModel;                  ///< This is a generic type which is used to distinguish between the different models!. See namespace ContactModels.
};

class ContactGraphNodeDataIteration : public ContactGraphNodeData {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataIteration()
    {
        m_LambdaBack.setZero();
        m_LambdaFront.setZero();

        m_b.setZero();

        m_u1BufferPtr = NULL; ///< Points to the velocity buffer only if the body is simulated
        m_u2BufferPtr = NULL; ///< Points to the velocity buffer only if the body is simulated

        m_bConverged = false; ///< Flag if convergence criteria is fulfilled, either InVelocityLocal, InLambda, InEnergyMix (with Lambda, and G_ii)
    }


    ~ContactGraphNodeDataIteration(){
    }

    FrontBackBuffer<VectorUObj,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> * m_u1BufferPtr; ///< Pointers into the right Front BackBuffer for bodies 1 and 2
    FrontBackBuffer<VectorUObj,FrontBackBufferPtrType::NoPtr, FrontBackBufferMode::NoConst> * m_u2BufferPtr; ///< Only valid for Simulated Objects


    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_LambdaBack;
    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_LambdaFront;

    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_R_i_inv_diag; // Build over G_ii
    Eigen::Matrix<PREC,Eigen::Dynamic,Eigen::Dynamic> m_G_ii; // just for R_ii, and maybee later for better solvers!

    Eigen::Matrix<PREC,Eigen::Dynamic,1> m_b;

    bool m_bConverged; ///< Converged either in LambdaLocal (lambdaNew to lambdaOld), or

   void swapVelocities() {

        if(m_u1BufferPtr){ m_u1BufferPtr->m_back.swap(m_u1BufferPtr->m_front); }

        if(m_u2BufferPtr){m_u2BufferPtr->m_back.swap(m_u2BufferPtr->m_front); }
    };

    void swapLambdas() {
        m_LambdaBack.swap(m_LambdaFront);
    };

};

/*
* The EdgeData class for the Contact Graph, nothing is deleted in this class, this is plain old data!
*/
class ContactGraphEdgeData {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphEdgeData(): m_pBody(NULL) {};

    RigidBodyType * m_pBody; // Tells to which body this edges belongs!

    //Eigen::Matrix<PREC,NDOFFriction+1,NDOFFriction+1> m_G_SE; // Start Node 1 to End Node 3 = G_13

};

template<typename ContactGraphMode > class ContactGraph;

struct ContactGraphMode{
    struct NoIteration{};
    struct ForIteration{};
};


struct ContactFeasibilityTable1{

/*                |------> i2
                       Locals  ||  Remotes
                   ==========================
                    sim|sta|ani||sim|sta|ani|
 _                 ==========================
 |    L   simulated| 1 | 1 | x || 1 | 0 | x |
 |    o      static| 1 | 0 | x || 1 | 0 | x |
 |    c.   animated| x | x | x || x | x | x |
 |        ---------------------------------------
 |    R   simulated| 1 | 1 | x || 0 | 0 | x |
 i1   e      static| 0 | 0 | x || 0 | 0 | x |
      m.   animated| x | x | x || x | x | x |

      1= feasible or allowed
      x= not implemented

*/

    DEFINE_RIGIDBODY_CONFIG_TYPES

    // 2D to 1D index with length L (store symetric matrix as 1D array)
    template<unsigned int L, char i1, char i2>
    struct make1DIndexSym{enum{result = i1*L +i2 - i1*(i1+1)/2 }};

    //Always zero
    template<unsigned int index> struct TableIndexFunc {
        enum { value = 0 };
    };
    // Contact combinations which are feasible value = 1
    //Specialization for Sim Local - Sim Local
    template<> struct TableIndexFunc
    <make1DIndexSym<RigidBodyType::BodyState::NSTATES, RigidBodyType::BodyState::SIMULATED, RigidBodyType::BodyState::SIMULATED>::result>
    {
        enum { value = 1 };
    };
    //Specialization for Sim Local - Static Local
    template<> struct TableIndexFunc
    <make1DIndexSym<RigidBodyType::BodyState::NSTATES,RigidBodyType::BodyState::SIMULATED, RigidBodyType::BodyState::STATIC>::result>
    {
        enum { value = 1 };
    };
    //Specialization for Sim Local - Sim Remote ( + RigidBodyType::BodyState::NSTATES)
    template<> struct TableIndexFunc
    <make1DIndexSym<RigidBodyType::BodyState::NSTATES,RigidBodyType::BodyState::SIMULATED, RigidBodyType::BodyState::SIMULATED + RigidBodyType::BodyState::NSTATES>::result>
    {
        enum { value = 1 };
    };
    //Specialization for Static Local - Sim Remote ( + RigidBodyType::BodyState::NSTATES)
    template<> struct TableIndexFunc
    <make1DIndexSym<RigidBodyType::BodyState::NSTATES,RigidBodyType::BodyState::STATIC, RigidBodyType::BodyState::SIMULATED + RigidBodyType::BodyState::NSTATES>::result>
    {
        enum { value = 1 };
    };

    typedef typename CompileTimeArray::generateArray<char,
            RigidBodyType::BodyState::NSTATES*(RigidBodyType::BodyState::NSTATES+1)/2,
            TableIndexFunc
            >::result Array;

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



    enum class NodeColor: int {LOCALNODE, REMOTENODE};

    ContactGraph(ContactParameterMap * contactParameterMap):
    m_nodeCounter(0),m_edgeCounter(0), m_nLambdas(0),m_nFrictionParams(0)
    {
        m_pContactParameterMap = contactParameterMap;
    }

    ~ContactGraph() {
        clearGraph();
    }

    void clearGraph() {
        // This deletes all nodes, edges
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

        //clear own lists
        m_localNodes.clear();
        m_remoteNodes.clear();
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
        bool isRemoteNode;
        bool feasible = checkFeasibilityOfContact(pCollData->m_pBody1, pCollData->m_pBody2 , isRemoteNode);

        if( pCollData->m_pBody1->m_pBodyInfo->m_isRemote || pCollData->m_pBody2->m_pBodyInfo->m_isRemote){
            //set the node color
            addedNode->m_nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::REMOTENODE);
            m_remoteNodes.push_back(addedNode);
        }else{
            //set the node color
            addedNode->m_nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::LOCALNODE);
            m_localNodes.push_back(addedNode);
        }

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
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    static const Eigen::Matrix<PREC,NDOFuObj,Eigen::Dynamic> * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }


    std::map<const RigidBodyType *, NodeListType > m_SimBodyToContactsList;
    typedef typename std::map<const RigidBodyType *, NodeListType >::iterator  BodyToContactsListIteratorType;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.
private:


    NodeListType m_remoteNodes; ///< These are the contact nodes which lie on the remote bodies
    NodeListType m_localNodes;  ///< These are the contact nodes which lie on the local bodies

    bool checkFeasibilityOfContact(RigidBodyType * p1, RigidBodyType * p2, bool & isRemoteNode){

        //Define the feasibility table
        typedef typename ContactFeasibilityTable1::Array ContactFeasibilityTable;

        // calculate table index
        char i1 = p1->m_eState;
        char i2 = p1->m_eState;

        // add offset if remote
        isRemoteNode = false;
        if(p1->m_pBodyInfo){
            if(p1->m_pBodyInfo->m_isRemote){
                i1 += RigidBodyType::BodyState::NSTATES
                isRemoteNode = true;
            }
        }
        if(p2->m_pBodyInfo){
            if(p2->m_pBodyInfo->m_isRemote){
                i2 += RigidBodyType::BodyState::NSTATES
                isRemoteNode = true;
            }
        }

        if(i1>i2){
            std::swap(i1,i2);
        }
        // Index into symetric array data of bools
        if(ContactFeasibilityTable::data[i1*NSTATES+i2  - i1*(i1+1)/2 ]

    }

    void computeParams(NodeDataType & nodeData) {
        if( nodeData.m_eContactModel == ContactModels::NCFContactModel ) {
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



        if(nodeData.m_eContactModel == ContactModels::NCFContactModel) {


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
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
class SorProxStepNodeVisitor{
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef ContactGraph<ContactGraphMode::ForIteration> ContactGraphType;
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

        if( nodeData.m_eContactModel == ContactModels::NCFContactModel ) {


            // Init the prox value
            nodeData.m_LambdaFront = nodeData.m_b;

            // FIRST BODY!
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {
                nodeData.m_LambdaFront += nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_front ;
            }
            // SECOND BODY!
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
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
                nodeData.m_LambdaFront.head<ContactModels::NormalAndCoulombFrictionContactModel::ConvexSet::Dimension>()
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
            if( nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED ) {
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
            if( nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {
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
        nodeData.m_b = nodeData.m_I_plus_eps.asDiagonal() * nodeData.m_xi;

        // u_0 , calculate const b
        // First Body
        if(nodeData.m_pCollData->m_pBody1->m_eState == RigidBodyType::SIMULATED) {

            // m_back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u1BufferPtr->m_front +=  nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body1 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body1.transpose() * nodeData.m_u1BufferPtr->m_back /* m_u_s */ ;
            nodeData.m_G_ii += nodeData.m_W_body1.transpose() * nodeData.m_pCollData->m_pBody1->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body1 ;
        }
        // SECOND BODY!
        if(nodeData.m_pCollData->m_pBody2->m_eState == RigidBodyType::SIMULATED ) {

            // m_back contains u_s + M^⁻1*h*deltaT already!
            nodeData.m_u2BufferPtr->m_front +=   nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * (nodeData.m_W_body2 * nodeData.m_LambdaBack ); /// + initial values M^⁻1 W lambda0 from percussion pool

            nodeData.m_b += nodeData.m_eps.asDiagonal() * nodeData.m_W_body2.transpose() *  nodeData.m_u1BufferPtr->m_back;
            nodeData.m_G_ii += nodeData.m_W_body2.transpose() * nodeData.m_pCollData->m_pBody2->m_MassMatrixInv_diag.asDiagonal() * nodeData.m_W_body2 ;
        }



        // Calculate R_ii
        nodeData.m_R_i_inv_diag(0) = m_alpha / (nodeData.m_G_ii(0,0));
        PREC r_T = m_alpha / ((nodeData.m_G_ii.diagonal().tail<2>()).maxCoeff());
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

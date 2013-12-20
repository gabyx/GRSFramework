#ifndef ContactGraphMPI_hpp
#define ContactGraphMPI_hpp

#include <boost/shared_ptr.hpp>
#include <unordered_map>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include RigidBody_INCLUDE_FILE

#include "GeneralGraph.hpp"
#include "CollisionData.hpp"
#include "ContactModels.hpp"
#include "ContactParameterMap.hpp"
#include "ContactGraphNodeDataMPI.hpp"

#include "InclusionSolverSettings.hpp"
#include "VectorToSkewMatrix.hpp"


template<typename Combo>
class ContactGraph : public Graph::GeneralGraph< ContactGraphNodeDataIteration,ContactGraphEdgeData > {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    typedef typename RigidBodyType::RigidBodyIdType RigidBodyIdType;

    typedef typename Combo::InclusionCommunicatorType InclusionCommunicatorType;
    typedef typename InclusionCommunicatorType::NeighbourMapType NeighbourMapType;

    typedef ContactGraphNodeDataIteration NodeDataType;
    typedef ContactGraphEdgeData EdgeDataType;
    typedef typename Graph::Node< NodeDataType, EdgeDataType> NodeType;
    typedef typename Graph::Edge< NodeDataType, EdgeDataType> EdgeType;

    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType NodeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType EdgeListType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType NodeListIteratorType;
    typedef typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType EdgeListIteratorType;

    typedef ContactGraphNodeDataSplitBody SplitBodyNodeDataType;
    typedef std::unordered_map<RigidBodyIdType, SplitBodyNodeDataType* > SplitBodyNodeDataListType;

    enum class NodeColor: unsigned short {LOCALNODE, REMOTENODE, SPLITNODE};

    ContactGraph(boost::shared_ptr<DynamicsSystemType> pDynSys);
    void setInclusionCommunicator(boost::shared_ptr<InclusionCommunicatorType> pInclusionComm);

    ~ContactGraph();

    void setLog(Logging::Log * solverLog);
    void clearGraph();
    void addNode(CollisionData * pCollData);



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
			vv.visitNode(*(*curr_node));
	}
	//Remote Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorRemote(TNodeVisitor & vv){
		for(auto curr_node = m_remoteNodes.begin(); curr_node != m_remoteNodes.end(); curr_node++)
			vv.visitNode(*(*curr_node));
	}

	//Remote Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorSplitBody(TNodeVisitor & vv){
		for(auto curr_node = m_splittedNodes.begin(); curr_node != m_splittedNodes.end(); curr_node++){
           vv.visitNode(*(curr_node->second));
		}
	}



    std::unordered_map<const RigidBodyType *, NodeListType > m_SimBodyToContactsList;

    unsigned int m_nLambdas; ///< The number of all scalar forces in the ContactGraph.
    unsigned int m_nFrictionParams; ///< The number of all scalar friction params in the ContactGraph.


    std::pair<SplitBodyNodeDataType *, bool> addSplitBodyNode(RigidBodyType * body, const RankIdType & rank);

    inline NodeListType & getLocalNodeListRef(){return m_localNodes;}
    inline NodeListType & getRemoteNodeListRef(){return m_remoteNodes;}
    inline SplitBodyNodeDataListType & getSplitBodyNodeListRef(){return m_splittedNodes;}

private:

    Logging::Log * m_pSolverLog;

    boost::shared_ptr<InclusionCommunicatorType> m_pInclusionComm;
    NeighbourMapType * m_pNbDataMap; ///< NeighbourMap to insert remote bodies which have contacts

    boost::shared_ptr<DynamicsSystemType> m_pDynSys;

    //std::map<unsigned int, NodeListType> m_nodeMap; //TODO make a map whith each color!
    NodeListType m_remoteNodes; ///< These are the contact nodes which lie on the remote bodies (ref to m_nodeMap)
    NodeListType m_localNodes;  ///< These are the contact nodes which lie on the local bodies (ref to m_nodeMap)

    SplitBodyNodeDataListType m_splittedNodes; ///< These are the billateral nodes between the splitted bodies in the contact graph


    void computeParams(NodeDataType & nodeData);

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


#include "ContactGraphMPI.icc"

#endif

#ifndef GMSF_Dynamics_Inclusion_ContactGraphMPI_hpp
#define GMSF_Dynamics_Inclusion_ContactGraphMPI_hpp

#include <memory>
#include <unordered_map>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"
#include "AssertionDebug.hpp"

#include "BitCount.hpp"

#include RigidBody_INCLUDE_FILE

#include "GeneralGraph.hpp"
#include "CollisionData.hpp"
#include "ContactModels.hpp"
#include "ContactParameterMap.hpp"
#include "ContactGraphNodeDataMPI.hpp"

#include InclusionSolverSettings_INCLUDE_FILE
#include "VectorToSkewMatrix.hpp"


template<typename TCombo>
class ContactGraph : public Graph::GeneralGraph< ContactGraphNodeDataIteration,ContactGraphEdgeData > {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using RigidBodyIdType = typename RigidBodyType::RigidBodyIdType;

    using NodeDataType = ContactGraphNodeDataIteration;
    using EdgeDataType = ContactGraphEdgeData;
    using NodeType = typename Graph::Node< NodeDataType, EdgeDataType>;
    using EdgeType = typename Graph::Edge< NodeDataType, EdgeDataType>;

    using NodeListType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListType;
    using EdgeListType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListType;
    using NodeListIteratorType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::NodeListIteratorType;
    using EdgeListIteratorType = typename Graph::GeneralGraph< NodeDataType,EdgeDataType >::EdgeListIteratorType;

    using SplitBodyNodeDataType = ContactGraphNodeDataSplitBody;
    typedef std::unordered_map<RigidBodyIdType, SplitBodyNodeDataType* > SplitBodyNodeDataListType;

    enum class NodeColor: unsigned short {LOCALNODE, REMOTENODE, SPLITNODE};

    using InclusionCommunicatorType = typename TCombo::InclusionCommunicatorType;
    using NeighbourMapType = typename InclusionCommunicatorType::NeighbourMapType;


    ContactGraph(std::shared_ptr<DynamicsSystemType> pDynSys);
    void setInclusionCommunicator(InclusionCommunicatorType * pInclusionComm);

    ~ContactGraph();

    void setLog(Logging::Log * solverLog);
    void clearGraph();
    void addNode(CollisionData * pCollData);

    void resetAfterOneIteration(unsigned int globalIterationCounter){
        m_maxResidual = 0;
    }

    inline  const MatrixUBodyDyn & getW_bodyRef(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1  == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  (nodeData.m_W_body1) :  (nodeData.m_W_body2);
    }

    inline  const MatrixUBodyDyn * getW_body(NodeDataType& nodeData, const RigidBodyType * pBody) {
        ASSERTMSG( nodeData.m_pCollData->m_pBody1 == pBody || nodeData.m_pCollData->m_pBody2  == pBody, " Something wrong with this node, does not contain the pointer: pBody!");
        return (nodeData.m_pCollData->m_pBody1 == pBody)?  &(nodeData.m_W_body1) :  &(nodeData.m_W_body2);
    }

    //Local Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorLocal(TNodeVisitor & vv){
		for(auto & node : m_localNodes)
			vv.visitNode(*(node));
	}
	//Remote Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorRemote(TNodeVisitor & vv){
		for(auto & node : m_remoteNodes)
			vv.visitNode(*(node));
	}

	//Remote Visitor
    template<typename TNodeVisitor>
	void applyNodeVisitorSplitBody(TNodeVisitor & vv){
		for(auto & node : m_splittedNodes){
          vv.visitNode(*(node.second));
		}
	}

	//void updateSplitBodyNode(RigidBodyIdType id , RankIdType rank, const VectorUBody & u);


    std::pair<SplitBodyNodeDataType *, bool> addSplitBodyNode(RigidBodyType * body, const RankIdType & rank);

    inline NodeListType & getLocalNodeListRef(){return m_localNodes;}
    inline NodeListType & getRemoteNodeListRef(){return m_remoteNodes;}
    inline SplitBodyNodeDataListType & getSplitBodyNodeListRef(){return m_splittedNodes;}

    /**
    * @brief Return true if the current contact problem is uncoupled from other processes
    */
    inline bool isUncoupled(){
        return (m_localNodes.size() >= 0
                && m_remoteNodes.size() == 0
                && m_splittedNodes.size() == 0 );
    }

    inline bool hasNoNodes(){
        return (m_localNodes.size() == 0
                && m_remoteNodes.size() == 0
                && m_splittedNodes.size() == 0 );
    }

    inline typename NodeListType::size_type getNLocalNodes(){return m_localNodes.size();}
    inline typename NodeListType::size_type getNRemoteNodes(){return m_remoteNodes.size();}
    inline typename NodeListType::size_type getNSplitBodyNodes(){return m_splittedNodes.size();}

    inline typename DynamicsSystemType::RigidBodyContainerType & getRemoteBodiesWithContactsListRef(){return m_remoteBodiesWithContacts;}
    inline typename DynamicsSystemType::RigidBodyContainerType & getLocalBodiesWithContactsListRef(){return m_localBodiesWithContacts;};

    unsigned int getNContactModelsUsed(){
        return BitCount::count(m_usedContactModels);
    }


    PREC m_maxResidual = 0;

private:

    Logging::Log * m_pSolverLog;

    using ContactModelEnumIntType = typename std::underlying_type<ContactModels::Enum>::type;
    ContactModelEnumIntType m_usedContactModels = 0; ///< Bitflags which mark all used contactmodels


    InclusionCommunicatorType * m_pInclusionComm;
    NeighbourMapType * m_pNbDataMap; ///< NeighbourMap to insert remote bodies which have contacts

    std::shared_ptr<DynamicsSystemType> m_pDynSys;

    //std::map<unsigned int, NodeListType> m_nodeMap; //TODO make a map whith each color!
    NodeListType m_remoteNodes; ///< These are the contact nodes which lie on the remote bodies (ref to m_nodeMap)
    NodeListType m_localNodes;  ///< These are the contact nodes which lie on the local bodies (ref to m_nodeMap)

    SplitBodyNodeDataListType m_splittedNodes; ///< These are the billateral nodes between the splitted bodies in the contact graph

    typename DynamicsSystemType::RigidBodyContainerType  m_remoteBodiesWithContacts; ///< This is our storage of all remote bodies which are in the contact graph
    typename DynamicsSystemType::RigidBodyContainerType  m_localBodiesWithContacts;

    template<int bodyNr, bool addEdges = true>
    void initNodeSimBody(NodeType * pNode, RigidBodyType * pBody,  bool isRemote);

    void setContactModel(NodeDataType & nodeData);

    ContactParameterMap* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    unsigned int m_nodeCounter = 0; ///< An node counter, starting at 0.
    unsigned int m_edgeCounter = 0; ///< An edge counter, starting at 0.



};


#include "ContactGraphMPI.icc"

#endif

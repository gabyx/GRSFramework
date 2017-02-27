// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ContactGraphMPI_hpp
#define GRSF_dynamics_inclusion_ContactGraphMPI_hpp

#include <memory>
#include <unordered_map>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/BitCount.hpp"

#include RigidBody_INCLUDE_FILE

#include "GRSF/dynamics/collision/CollisionData.hpp"
#include "GRSF/dynamics/inclusion/ContactGraphNodeDataMPI.hpp"
#include "GRSF/dynamics/inclusion/ContactGraphVisitors.hpp"
#include "GRSF/dynamics/inclusion/ContactModels.hpp"
#include "GRSF/dynamics/inclusion/ContactParameterMap.hpp"
#include "GRSF/dynamics/inclusion/GeneralGraph.hpp"

#include InclusionSolverSettings_INCLUDE_FILE
#include "GRSF/dynamics/general/VectorToSkewMatrix.hpp"

using ContactGraphTraits =
    Graph::GraphTraitsSymmetric<meta::list<ContactGraphNodeDataUCF, ContactGraphNodeDataSplitBody>,
                                meta::list<ContactGraphEdgeData, void, void>>;

template <typename TCombo>
class ContactGraph : public Graph::GeneralGraph<ContactGraphTraits>
{
public:
    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    DEFINE_CONTACT_GRAPH_VISITORS_AS_FRIEND

    using RigidBodyContainerType = typename DynamicsSystemType::RigidBodyContainerType;
    using RigidBodyIdType        = typename RigidBodyType::RigidBodyIdType;

    using UCFNodeDataType       = ContactGraphNodeDataUCF;
    using SplitBodyNodeDataType = ContactGraphNodeDataSplitBody;
    using CommonEdgeDataType    = ContactGraphEdgeData;

    using SplitBodyNodeDataMapType = std::unordered_map<RigidBodyIdType, SplitBodyNodeDataType*>;

    using InclusionCommunicatorType = typename TCombo::InclusionCommunicatorType;
    using NeighbourMapType          = typename InclusionCommunicatorType::NeighbourMapType;

    enum class NodeColor : unsigned short
    {
        LOCALNODE,
        REMOTENODE,
        SPLITNODE
    };

    ContactGraph(std::shared_ptr<DynamicsSystemType> pDynSys);
    void setInclusionCommunicator(InclusionCommunicatorType* pInclusionComm);

    ~ContactGraph();

    void setLog(Logging::Log* solverLog);
    void clearGraph();

    template <bool addEdges>
    void addNode(CollisionData* pCollData);

    void resetAfterOneIteration(unsigned int globalIterationCounter)
    {
        m_maxResidual = 0;
    }

    std::pair<SplitBodyNodeDataType*, bool> addSplitBodyNode(RigidBodyType* body, const RankIdType& rank);

    inline SplitBodyNodeDataMapType& getSplitBodyNodeListRef()
    {
        return m_splittedNodes;
    }

    /**
    * @brief Return true if the current contact problem is uncoupled from other processes
    */
    inline bool isUncoupled()
    {
        return (m_nodeCounterLocal >= 0 && m_nodeCounterRemote == 0 && m_nodeCounterSplitBody == 0);
    }

    inline bool hasNoNodes()
    {
        return (m_nodeCounterLocal == 0 && m_nodeCounterRemote == 0 && m_nodeCounterSplitBody == 0);
    }

    inline std::size_t getNLocalNodes()
    {
        return m_nodeCounterLocal;
    }
    inline std::size_t getNRemoteNodes()
    {
        return m_nodeCounterRemote;
    }
    inline std::size_t getNSplitBodyNodes()
    {
        return m_nodeCounterSplitBody;
    }

    inline RigidBodyContainerType& getRemoteBodiesWithContactsListRef()
    {
        return m_remoteBodiesWithContacts;
    }
    inline RigidBodyContainerType& getLocalBodiesWithContactsListRef()
    {
        return m_localBodiesWithContacts;
    };

    unsigned int getNContactModelsUsed()
    {
        return BitCount::count(m_usedContactModels);
    }

    inline PREC getMaxResidual()
    {
        return m_maxResidual;
    }

    inline void reserveNodes(std::size_t nodes, std::size_t splitbodyNodes)
    {
        GeneralGraph::reserveNodes<UCFNodeDataType>(nodes);
        GeneralGraph::reserveNodes<SplitBodyNodeDataType>(splitbodyNodes);
    }

    template <typename Visitor>
    void visitNormalNodes(Visitor&& v)
    {
        this->visitNodes<UCFNodeDataType>(v);
    }

    template <typename Visitor>
    void visitSplitNodes(Visitor&& v)
    {
        this->visitNodes<SplitBodyNodeDataType>(v);
    }

private:
    Logging::Log* m_pSolverLog;

    using ContactModelEnumIntType               = typename std::underlying_type<ContactModels::Enum>::type;
    ContactModelEnumIntType m_usedContactModels = 0;  ///< Bitflags which mark all used contactmodels

    bool m_firstIteration = true;
    PREC m_maxResidual    = 0.0;

    InclusionCommunicatorType* m_pInclusionComm;
    NeighbourMapType*          m_pNbDataMap;  ///< NeighbourMap to insert remote bodies which have contacts

    std::shared_ptr<DynamicsSystemType> m_pDynSys;

    SplitBodyNodeDataMapType
        m_splittedNodes;  ///< These are the billateral nodes between the splitted bodies in the contact graph

    RigidBodyContainerType
                           m_remoteBodiesWithContacts;  ///< This is our storage of all remote bodies which are in the contact graph
    RigidBodyContainerType m_localBodiesWithContacts;

    using NodeListType     = std::vector<Graph::NodeBase<Traits>*>;
    using SimBodyToNodeMap = std::unordered_map<const RigidBodyType*, NodeListType>;
    SimBodyToNodeMap m_simBodiesToContactsMap;

    friend struct NodeDataInit;
    friend struct NodeInit;

    struct NodeDataInit
    {
        NodeDataInit(ContactGraph* p) : m_p(p)
        {
        }
        ContactGraph* m_p = nullptr;

        void apply(UCFNodeDataType& nodeData)
        {
            const unsigned int dimSet = ContactModels::getLambdaDim(ContactModels::Enum::UCF);
            // nodeData.m_eps.setZero(dimSet);
            nodeData.m_chi.setZero(dimSet);

            // Set epsilon  values
            using CMT         = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            nodeData.m_eps(0) = nodeData.m_contactParameter.m_params[CMT::epsNIdx];
            nodeData.m_eps(1) = nodeData.m_contactParameter.m_params[CMT::epsTIdx];
            nodeData.m_eps(2) = nodeData.m_eps(1);
        }
    };
    struct NodeInit
    {
        NodeInit(ContactGraph* p) : m_p(p), m_nodeDataInit(p)
        {
        }

        ContactGraph* m_p = nullptr;
        NodeDataInit  m_nodeDataInit;

        template <bool addEdges, typename TNode, typename TCollData, typename TContactParams, typename TRemotePair>
        void apply(TNode* pNode, TCollData* pCollData, TContactParams& contactParams, TRemotePair& isRemote)
        {
            auto& nodeData = pNode->getData();

            nodeData.m_pCollData        = pCollData;
            nodeData.m_contactParameter = contactParams;

            if (isRemote.first || isRemote.second)
            {
                // Remote - Remote or Remote-Local Contacts

                if (!isRemote.first or !isRemote.second)
                {
                    // Remote-Local or Remote-Remote
                    // set the node color
                    nodeData.m_nodeColor = EnumConversion::toIntegral(NodeColor::REMOTENODE);
                }
                else
                {
                    // Remote-Remote contact
                    nodeData.m_nodeColor = EnumConversion::toIntegral(NodeColor::REMOTENODE);
                }
                ++m_p->m_nodeCounterRemote;
            }
            else
            {
                // Local - Local Contacts
                // set the node color
                nodeData.m_nodeColor = static_cast<unsigned int>(NodeColor::LOCALNODE);
                ++m_p->m_nodeCounterLocal;
            }

            m_nodeDataInit.apply(nodeData);

            // FIRST BODY!
            RigidBodyType* pBody = pCollData->m_pBody[0];
            if (pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED)
            {
                initNodeSimBody<1, addEdges>(pNode, nodeData, pBody, isRemote.first);
            }
            else if (pBody->m_eMode == RigidBodyType::BodyMode::ANIMATED)
            {
                GRSF_ERRORMSG("ContactGraph:: Animated body, node init not implemented")
            }

            // SECOND BODY!
            pBody = pCollData->m_pBody[1];
            if (pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED)
            {
                initNodeSimBody<2, addEdges>(pNode, nodeData, pBody, isRemote.second);
            }
            else if (pBody->m_eMode == RigidBodyType::BodyMode::ANIMATED)
            {
                GRSF_ERRORMSG("ContactGraph:: Animated body, node init not implemented")
            }
        }

        template <typename TEdge>
        struct AddEdgeIn
        {
            template <typename TNode>
            inline void operator()(TNode& node)
            {
                dispatch<typename TNode::NodeDataType>::apply(node, m_p);
            }
            // In general do nothing! (Partial spezialization works in nested classes, full spezialization doesnt ->
            // dummy)
            template <typename NodeDataType, typename Dummy = void>
            struct dispatch
            {
                template <typename TNode>
                static inline void apply(TNode& node, TEdge* e)
                {
                }
            };

            // Only add edge to UCFNodeDataTypes, others are set to void in GraphTraits and no edge is supported
            template <typename Dummy>
            struct dispatch<UCFNodeDataType, Dummy>
            {
                template <typename TNode>
                static inline void apply(TNode& node, TEdge* e)
                {
                    node.addEdgeIn(e);
                }
            };

            TEdge* m_p;
        };

        template <int bodyNr, bool addEdges, typename TNode, typename TNodeData>
        void initNodeSimBody(TNode* pNode, TNodeData& nodeData, RigidBodyType* pBody, bool isRemote)
        {
            // Add to the corresponding bodyWithContacts list
            if (isRemote)
            {
                LOGSLLEVEL2_CONTACT(m_p->m_pSolverLog,
                                    "\t---> Remote body id: " << RigidBodyId::getBodyIdString(pBody->m_id)
                                                              << std::endl;)

                // Add to the neighbour data if remote contact
                if (nodeData.m_nodeColor == EnumConversion::toIntegral(NodeColor::REMOTENODE))
                {
                    auto* nbData = m_p->m_pNbDataMap->getNeighbourData(pBody->m_pBodyInfo->m_ownerRank);
                    GRSF_ASSERTMSG(nbData, "No neighbour data for rank " << pBody->m_pBodyInfo->m_ownerRank)
                    nbData->addRemoteBodyData(pBody);
                    // if this body is already added it does nothing!
                }
                else
                {
                    GRSF_ERRORMSG("Something wrong with node color!")
                }

                m_p->m_remoteBodiesWithContacts.addBody(pBody);
            }
            else
            {
                m_p->m_localBodiesWithContacts.addBody(pBody);
            }

            // Set Flag that this Body is in ContactGraph
            pBody->m_pSolverData->m_bInContactGraph = true;
            // Unset the flag when this Node is removed;

            // Link to FrontBackBuffer
            if (bodyNr == 1)
            {
                nodeData.m_uBufferPtr[0] = &pBody->m_pSolverData->m_uBuffer;
            }
            else
            {
                nodeData.m_uBufferPtr[1] = &pBody->m_pSolverData->m_uBuffer;
            }

            if (addEdges)
            {
                auto& nodesOnBody = m_p->m_simBodiesToContactsMap[pBody];

                // Add self edge! ===========================================================
                auto* addedEdge              = m_p->emplaceEdgeBack<CommonEdgeDataType>(m_p->m_edgeCounter);
                addedEdge->getData().m_pBody = pBody;

                // add links
                addedEdge->setStartNode(pNode);
                addedEdge->setEndNode(pNode);
                addedEdge->setTwinEdge(addedEdge);  // Current we dont need a twin edge, self referencing!
                // Add the edge to the nodes edge list!
                pNode->addEdgeOut(addedEdge);
                m_p->m_edgeCounter++;
                // ===========================================================================

                // iterate over the nodeList and add edges!
                // if no contacts are already on the body we skip this
                for (auto& pN : nodesOnBody)
                {
                    addedEdge                    = m_p->emplaceEdgeBack<CommonEdgeDataType>(m_p->m_edgeCounter);
                    addedEdge->getData().m_pBody = pBody;
                    // add link
                    addedEdge->setStartNode(pNode);
                    addedEdge->setEndNode(pN);
                    addedEdge->setTwinEdge(addedEdge);  // Current we dont need a twin edge, self referencing!
                    // Add the edge to the nodes edge list!
                    pNode->addEdgeOut(addedEdge);
                    pN->visit(AddEdgeIn<typename std::remove_pointer<decltype(addedEdge)>::type>{addedEdge});

                    m_p->m_edgeCounter++;
                }

                // Add new Node to the list;
                nodesOnBody.push_back(pNode);
            }
        }
    };

    ContactParameterMap*
        m_pContactParameterMap;  ///< A contact parameter map which is used to get the parameters for one contact.

    std::size_t m_nodeCounter = 0;  ///< An node counter, starting at 0.

    std::size_t m_nodeCounterLocal     = 0;  ///< An node counter, starting at 0.
    std::size_t m_nodeCounterRemote    = 0;  ///< An node counter, starting at 0.
    std::size_t m_nodeCounterSplitBody = 0;  ///< An node counter, starting at 0.

    std::size_t m_edgeCounter = 0;  ///< An edge counter, starting at 0.
};

#include "GRSF/dynamics/inclusion/ContactGraphMPI.icc"

#endif

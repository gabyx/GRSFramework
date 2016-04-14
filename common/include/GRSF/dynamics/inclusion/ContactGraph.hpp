// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ContactGraph_hpp
#define GRSF_dynamics_inclusion_ContactGraph_hpp


//#define SRUTIL_DELEGATE_PREFERRED_SYNTAX
#include <srutil/delegate/delegate.hpp> // Use fast SR delegates

/** Contact Graph */
#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/Asserts.hpp"

#include "GRSF/common/BitCount.hpp"
#include "GRSF/common/BitRepresentation.hpp"
#include "GRSF/common/EnumClassHelper.hpp"

#include "GRSF/dynamics/inclusion/GeneralGraph.hpp"
#include "GRSF/dynamics/collision/CollisionData.hpp"
#include "GRSF/dynamics/inclusion/ContactParameterMap.hpp"

#include "GRSF/dynamics/inclusion/ContactModels.hpp"
#include "GRSF/dynamics/inclusion/ContactGraphNodeData.hpp"

#include "GRSF/dynamics/inclusion/ContactGraphVisitors.hpp"


using ContactGraphTraits = Graph::GraphTraitsSymmetric<
                           meta::list<ContactGraphNodeDataUCF> ,
                           meta::list<ContactGraphEdgeData>
                           >;

/** Contact Graph which can be used to iterate over the nodes */
// #define RESIDUAL_SORTED_ITERATION   ///< Define this macro here to get a residual sorted iteration for SOR_CONTACT
class ContactGraph: public Graph::GeneralGraph< ContactGraphTraits > {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    DEFINE_CONTACT_GRAPH_VISITORS_AS_FRIEND

    using UCFNodeDataType = ContactGraphNodeDataUCF;
    using CommonEdgeDataType = ContactGraphEdgeData;

    ContactGraph(ContactParameterMap * contactParameterMap)
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

    ~ContactGraph() {
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
    void applyNodeVisitorSpecial(TNodeVisitor & vv) {

#ifdef RESIDUAL_SORTED_ITERATION
        if( m_firstIteration ) {
            for(auto curr_node = this->m_nodes.begin(); curr_node != this->m_nodes.end(); curr_node++) {
                vv.visitNode(*(*curr_node));
            }
            m_firstIteration = false;
        } else {
            for(auto curr_node = m_nodesBackRes->begin(); curr_node != m_nodesBackRes->end(); curr_node++) {
                vv.visitNode(*curr_node->second);
            }
        }
#else // No residual sorted iteration!
        this->visitNodes(vv);
#endif // RESIDUAL_SORTED_ITERATION

    }

    void resetAfterOneIteration(unsigned int globalIterationCounter) {

#ifdef RESIDUAL_SORTED_ITERATION
        // Switch potiner of residual list;
        if( globalIterationCounter % 1 == 0 ) {
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
        GeneralGraph::clear();

        m_edgeCounter = 0;
        m_nodeCounter = 0;

        m_simBodiesToContactsMap.clear();

        m_usedContactModels = 0;
        m_maxResidual = 0.0;
    }

    template<bool addEdges = true>
    void addNode(CollisionData * pCollData) {

        static NodeInit nodeInit(this);

        GRSF_ASSERTMSG(pCollData->m_pBody[0] != nullptr && pCollData->m_pBody[1] != nullptr, " Bodys are null pointers?");
        // cout << "add node : "<<m_nodeCounter<< " body id:" << RigidBodyId::getBodyIdString(pCollData->m_pBody[0]) <<" and "<< RigidBodyId::getBodyIdString(pCollData->m_pBody[1]) <<endl;
        // std::cout <<"b1:" << pCollData->m_pBody[0]->m_q_KI<< std::endl << pCollData->m_pBody[0]->m_r_S<< std::endl;;
        // std::cout <<"b2:" << pCollData->m_pBody[1]->m_q_KI << std::endl << pCollData->m_pBody[1]->m_r_S<< std::endl;;;
        //  add a contact node to the graph
        // check to which nodes we need to connect?
        // all nodes (contacts) which are in the BodyContactList (maps bodies -> contacts)

        // get contact parameters
        auto & contactParams = m_pContactParameterMap->getContactParams(pCollData->m_pBody[0]->m_eMaterial,
                               pCollData->m_pBody[1]->m_eMaterial);

        // Set flag for the corresponding model
        m_usedContactModels |= 1 << EnumConversion::toIntegral(contactParams.m_contactModel);

        // depending on type allocate corresponding node type
        if( contactParams.m_contactModel == ContactModels::Enum::UCF  ||
                contactParams.m_contactModel == ContactModels::Enum::UCFD ||
                contactParams.m_contactModel == ContactModels::Enum::UCFDD) {
            // add the pNodeData to the node list
            auto * addedNode = this->emplaceNodeBack<UCFNodeDataType>(m_nodeCounter);
            nodeInit.apply<addEdges>(addedNode, pCollData, contactParams);

        } else {
            GRSF_ASSERTMSG(false," You specified a contact model which has not been implemented so far!");
        }

        m_nodeCounter++;
    }

    using NodeListType = std::vector< Graph::NodeBase<Traits>* >;
    using SimBodyToNodeMap = std::unordered_map<const RigidBodyType *, NodeListType >;
    SimBodyToNodeMap m_simBodiesToContactsMap;

    unsigned int getNContactModelsUsed() {
        return BitCount::count<ContactModelEnumIntType>(m_usedContactModels);
    }

    inline PREC getMaxResidual() {
        return m_maxResidual;
    }

private:

    ContactParameterMap* m_pContactParameterMap; ///< A contact parameter map which is used to get the parameters for one contact.

    std::size_t m_nodeCounter = 0; ///< An node counter, starting at 0.
    std::size_t m_edgeCounter = 0; ///< An edge counter, starting at 0.

    bool m_firstIteration = true;
    PREC m_maxResidual = 0.0;

    using ContactModelEnumIntType = typename std::underlying_type<ContactModels::Enum>::type;
    ContactModelEnumIntType m_usedContactModels; ///< Bitflags which mark all used contactmodels

    friend struct NodeDataInit;
    friend struct NodeInit;

    struct NodeDataInit {

        NodeDataInit(ContactGraph * p): m_p(p) {}
        ContactGraph * m_p = nullptr;

        void apply(UCFNodeDataType & nodeData) {
            const unsigned int dimSet = ContactModels::getLambdaDim(ContactModels::Enum::UCF);
            //nodeData.m_eps.setZero(dimSet);
            nodeData.m_chi.setZero(dimSet);

            // Set epsilon  values
            using CMT = typename CONTACTMODELTYPE(ContactModels::Enum::UCF);
            nodeData.m_eps(0) = nodeData.m_contactParameter.m_params[CMT::epsNIdx];
            nodeData.m_eps(1) = nodeData.m_contactParameter.m_params[CMT::epsTIdx];
            nodeData.m_eps(2) = nodeData.m_eps(1);
        }

    };
    struct NodeInit {

        NodeInit(ContactGraph * p): m_p(p), m_nodeDataInit(p) {}

        ContactGraph * m_p = nullptr;
        NodeDataInit m_nodeDataInit;


        template<bool addEdges, typename TNode, typename TCollData, typename TContactParams>
        void apply(TNode * pNode,
                   TCollData * pCollData ,
                   TContactParams & contactParams) {

            auto & nodeData = pNode->getData();

            nodeData.m_pCollData = pCollData;
            nodeData.m_contactParameter = contactParams;

            m_nodeDataInit.apply(nodeData);

            // FIRST BODY!
            RigidBodyType * pBody = pCollData->m_pBody[0];
            if( pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                initNodeSimBody<1,addEdges>(pNode,nodeData,pBody);
            } else if(pBody->m_eMode == RigidBodyType::BodyMode::ANIMATED ) {
                GRSF_ERRORMSG("ContactGraph:: Animated body, node init not implemented")
            }

            // SECOND BODY!
            pBody = pCollData->m_pBody[1];
            if( pBody->m_eMode == RigidBodyType::BodyMode::SIMULATED ) {
                initNodeSimBody<2,addEdges>(pNode,nodeData,pBody);
            } else if(pBody->m_eMode == RigidBodyType::BodyMode::ANIMATED ) {
                GRSF_ERRORMSG("ContactGraph:: Animated body, node init not implemented")
            }

        }

        template<typename TEdge>
        struct AddEdgeIn {
            template<typename TNode>
            inline void operator()(TNode & node) {
                node.addEdgeIn(m_p);
            }
            TEdge * m_p;
        };

        template<int bodyNr, bool addEdges, typename TNode , typename TNodeData>
        void initNodeSimBody(TNode * pNode,
                             TNodeData & nodeData,
                             RigidBodyType * pBody) {

            //Set Flag that this Body is in ContactGraph
            pBody->m_pSolverData->m_bInContactGraph = true;
            // Unset the flag when this Node is removed;

            //Link to FrontBackBuffer
            if(bodyNr==1) {
                nodeData.m_uBufferPtr[0] = & pBody->m_pSolverData->m_uBuffer;
            } else {
                nodeData.m_uBufferPtr[1] = & pBody->m_pSolverData->m_uBuffer;
            }


            auto & nodesOnBody = m_p->m_simBodiesToContactsMap[pBody];

            if( addEdges ) {
                // Add self edge! ===========================================================
                auto * addedEdge = m_p->emplaceEdgeBack<CommonEdgeDataType>(m_p->m_edgeCounter);
                addedEdge->getData().m_pBody = pBody;

                // add links
                addedEdge->setStartNode(pNode);
                addedEdge->setEndNode(pNode);
                addedEdge->setTwinEdge(addedEdge); // Current we dont need a twin edge, self referencing!
                // Add the edge to the nodes edge list!
                pNode->addEdgeOut( addedEdge );
                m_p->m_edgeCounter++;
                // ===========================================================================

                // iterate over the nodeList and add edges!
                // if no contacts are already on the body we skip this
                for(auto & pN : nodesOnBody) {

                    addedEdge = m_p->emplaceEdgeBack<CommonEdgeDataType>(m_p->m_edgeCounter);
                    addedEdge->getData().m_pBody = pBody;
                    // add link
                    addedEdge->setStartNode(pNode);
                    addedEdge->setEndNode(pN);
                    addedEdge->setTwinEdge(addedEdge); // Current we dont need a twin edge, self referencing!
                    // Add the edge to the nodes edge list!
                    pNode->addEdgeOut( addedEdge );
                    pN->visit( AddEdgeIn<typename  std::remove_pointer<decltype(addedEdge)>::type> {addedEdge} );

                    m_p->m_edgeCounter++;
                }
            }

            // Add new Node to the list;
            nodesOnBody.push_back(pNode);
        }

    };

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

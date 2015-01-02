#ifndef GRSF_Logic_ExecutionTreeInOut_hpp
#define GRSF_Logic_ExecutionTreeInOut_hpp

#include <algorithm>
#include <unordered_set>

#include <boost/mpl/vector.hpp>

#include "GRSF/Logic/LogicNode.hpp"
#include "GRSF/Logic/LogicSocket.hpp"





class ExecutionTreeInOut {
private:
public:

    using NodeListT = std::vector<LogicNode*>;
    using NodeMapT = std::unordered_map<unsigned int, LogicNode*>;
    using NodeSetT = std::unordered_set<LogicNode*>;
    using GroupNodeMapT = std::unordered_map<unsigned int, NodeSetT >;
    using GroupExecutionListT = std::unordered_map<unsigned int, NodeListT >;
    using OutputNodeMapT = NodeSetT;

    ExecutionTreeInOut() {
    }

    ~ExecutionTreeInOut() {
        for(auto & n : m_nodeMap) {
            delete n.second;
        }
    }

    void setInputNode(unsigned int id){
        setInOutNode<true>(id);
    }
    NodeSetT & getInputNode() {
        return m_inputNodes;
    };

    void setOutputNode(unsigned int id){
        setInOutNode<false>(id);
    }

    OutputNodeMapT & getOutputNodes(){
        return m_outputNodes;
    }

    virtual void addNode(LogicNode * node, bool isInput = false, bool isOutput = false) {
        if(isInput && isOutput){ ERRORMSG("Wrong arguements!")}


        // Add to global map
        auto res = m_nodeMap.emplace(node->m_id, node);
        if(!res.second) {
            ERRORMSG("Node id: " << node->m_id << " already in map");
        }

        // Add to input or output
        if(isInput){
             setInOutNode<true>(node->m_id);
        }else if(isOutput){
             setInOutNode<false>(node->m_id);
        }

    }

    virtual void addNodeToGroup(unsigned int nodeId, unsigned int groupId){

        auto inNodeIt = m_nodeMap.find(nodeId);
        if(inNodeIt == m_nodeMap.end()){
            ERRORMSG("Node with id: " << nodeId << " has not been added to the tree!")
        }
        // Add to group
        auto res = m_groupNodes[groupId].insert(inNodeIt->second);
        if(!res.second){
            ERRORMSG("Node id: " << inNodeIt->second->m_id << " already in group: " << groupId);
        }
    }

    virtual void makeGetLink(unsigned int outN, unsigned int outS,
                             unsigned int inN, unsigned int inS)
    {
         auto outNit = m_nodeMap.find(outN);
         auto inNit = m_nodeMap.find(inN);
         if(outNit == m_nodeMap.end() || inNit == m_nodeMap.end() ){
            ERRORMSG("Node: " << outN << " or " << inN << " does not exist!")
         }
         LogicNode::makeGetLink(outNit->second,outS,inNit->second,inS);
    }

    virtual void makeWriteLink(unsigned int outN, unsigned int outS,
                               unsigned int inN, unsigned int inS)
    {
         auto outNit = m_nodeMap.find(outN);
         auto inNit = m_nodeMap.find(inN);
         if(outNit == m_nodeMap.end() || inNit == m_nodeMap.end() ){
            ERRORMSG("Node: " << outN << " or " << inN << " does not exist!")
         }
         LogicNode::makeWriteLink(outNit->second,outS,inNit->second,inS);
    }


    /** Init group */
    virtual void initialize(unsigned int groupId) {
        for(auto & n : m_groupNodes[groupId]) {
            n->initialize();
        }
    }

    /** Execute group */
    virtual void execute(unsigned int groupId) {
        for(auto & n : m_groupExecList[groupId]) {
            n->compute();
        }
    }

    /** Execute all groups */
    virtual void executeAll() {
        for(auto &  g : m_groupExecList){
            for(auto & n : g.second) {
                n->compute();
            }
        }
    }

    /** Init all groups */
    virtual void initializeAll() {
        for(auto &  g : m_groupExecList){
            for(auto & n : g.second) {
                n->initialize();
            }
        }
    }

    virtual void setup() {

        if(m_outputNodes.size()== 0) {
            ERRORMSG("No output node specified")
        }

        // Solve execution order for every group list!
        ExecutionOrderSolver s;
        m_groupExecList.clear();
        for(auto & p : m_groupNodes){

            // fill nodes into execution list
            auto & l = m_groupExecList[p.first];
            std::for_each(p.second.begin(),p.second.end(),
                 [&l](const typename NodeSetT::value_type& n)
                 { l.push_back(n); });

            s.solve(l, l);

        }

        // Check if input is reachable from all outputs
        m_inputNotReachable = false;
        ReachNodeCheck c;
        for(auto & o : m_outputNodes){
                for(auto & i : m_inputNodes){
                     m_inputNotReachable = m_inputNotReachable && !c.check(o, i);
                }
        }
        if(m_inputNotReachable) {
            ERRORMSG("Your input nodes cannot be reached by all output node!")
        }

    }


    std::string getExecutionOrderInfo(std::string suffix="\t\t"){
        // Print execution order
        std::stringstream s;
        for(auto &g : m_groupExecList){
            s << suffix << "Execution order for group id: " <<g.first << std::endl;
            s << suffix << "NodeId\t|\tPriority  "<< std::endl;
                for(auto n : g.second){
                    s << suffix << Utilities::stringFormat("%4i \t|\t %4i", n->m_id , n->getPriority()) <<std::endl;
                }
            s << suffix << "==============================" <<std::endl;
        }
        return s.str();
    }

protected:

    class ExecutionOrderSolver{
    public:

        void solve(NodeListT & c,
                   NodeListT & orderedNodes){


            // Solve Execution order,
            // start a depth first search recursion for all nodes in c which determines an execution order by setting the priority
            for(auto & p : c){
                std::unordered_set<unsigned int> nodesCurrDepth;
                // start recursion from this node
                solveRec(p,nodesCurrDepth);
            }

            // Sort all nodes according to priority (asscending) (lowest is most important)
            std::sort(orderedNodes.begin(), orderedNodes.end(),
            [](LogicNode* const & a, LogicNode* const &b) {
                return a->getPriority() < b->getPriority();
            }
                     );

            // Invert priority such that lowest is now the highest and the first one!
            auto maxPrio = orderedNodes.back()->getPriority();
            for(auto n : orderedNodes) {
                n->setPriority( -n->getPriority() + maxPrio);
            }

        }

    private:

        /**
        * Breath first search: this function returns recursively the priority
        */

        unsigned int solveRec( LogicNode* node,
                                             std::unordered_set<unsigned int> & nodesCurrDepth) {

            nodesCurrDepth.insert(node->m_id);

            //visit all input sockets and their node!
            auto & inSockets = node->getInputs();
            for(auto * s : inSockets) {
                if(s->isLinked()) {

                    auto * fsock = s->getFrom();

                    auto * adjNode = fsock->getParent();
                    if(nodesCurrDepth.find(adjNode->m_id)!=nodesCurrDepth.end()) {
                        ERRORMSG("Your execution logic graph contains a cylce from node: " << node->m_id << " socket: " << s->m_id <<
                                 " to node: " << adjNode->m_id << " socket: " << fsock->m_id);
                    }

                    unsigned int prioAdj = solveRec(adjNode,nodesCurrDepth);

                    // Set the current node to the priority of the below tree +1
                    node->setPriority( std::max(  prioAdj+1, node->getPriority() ) );
                }
            }

            nodesCurrDepth.erase(node->m_id);

            return node->getPriority();

        }
        bool m_inputReachable = false;
        LogicNode * m_reachNode = nullptr;
    };

    // Only for directed graphs, does not detect cycles -> endless loop!
    class ReachNodeCheck{
    public:
        // From end to start node
        bool check(LogicNode * endNode, LogicNode * startNode){

            if( endNode == startNode ){return true;}

            m_start = startNode;
            m_reached = false;

            std::deque<LogicNode *> currentNodes; // Breath first search
            currentNodes.push_back(endNode);

            // visit current front node, as long as currentNode list is not empty or start has not yet been found!
            while(currentNodes.size() != 0 && m_reached == false ){
                visit(currentNodes.front(),currentNodes);
                currentNodes.pop_front();
            }

            return m_reached;
        }

    private:

        /**
        * Breath first search:
        */
        void visit( LogicNode* node, std::deque<LogicNode *> & queu ) {

            //visit all input sockets and their node!
            auto & inSockets = node->getInputs();
            for(auto * s : inSockets) {
                if(s->isLinked()) {

                    auto * fsock = s->getFrom();
                    ASSERTMSG(fsock,"linked but from ptr null");

                    auto * adjNode = fsock->getParent();
                    ASSERTMSG(adjNode,"Adj node null");

                    // If we reached the start node, return!
                    if( m_start == adjNode){
                        m_reached = true;
                        return;
                    }

                    queu.push_back(adjNode);
                }
            }
        }

        bool m_reached = false;
        LogicNode * m_start = nullptr;
    };



    template<bool input>
    void setInOutNode(unsigned int id){
        // Set output node
        auto it = m_nodeMap.find(id);
        if(it == m_nodeMap.end()) {
            ERRORMSG("No output node with id: " << id << " found " << std::endl)
        }
        if(input){
            m_inputNodes.emplace(it->second);
        }else{
            m_outputNodes.emplace(it->second);
        }

    }

    bool m_inputNotReachable = true;

    NodeSetT m_inputNodes;  ///< the input nodes
    OutputNodeMapT m_outputNodes; ///< all output nodes in the tree

    NodeMapT  m_nodeMap; ///< all nodes in the tree

    // Additional group list
    GroupNodeMapT m_groupNodes;

    GroupExecutionListT m_groupExecList;

};

#endif // ExecutionTreeInOut_hpp


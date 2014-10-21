#ifndef ExecutionTreeInOut_hpp
#define ExecutionTreeInOut_hpp

#include <algorithm>
#include <unordered_set>

#include <boost/mpl/vector.hpp>

#include "LogicNode.hpp"
#include "LogicSocket.hpp"





class ExecutionTreeInOut {
private:
public:

    using OutputNodeMap = std::unordered_map<unsigned int, LogicNode*>;

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
    LogicNode* getInputNode() {
        return m_inputNode;
    };

    void setOutputNode(unsigned int id){
        setInOutNode<false>(id);
    }

    OutputNodeMap & getOutputNodes(){
        return m_outputNodes;
    }

    void addNode(LogicNode * node, bool isInput = false, bool isOutput = false) {
        if(isInput && isOutput){ ERRORMSG("Wrong arguements!")}
        auto res = m_nodeMap.emplace(node->m_id, node);
        if(res.second) {
            m_nodes.push_back(node);
        } else {
            ERRORMSG("Node id: " << node->m_id << " already in map");
        }

        if(isInput){
             setInOutNode<true>(node->m_id);
        }else if(isOutput){
             setInOutNode<false>(node->m_id);
        }

    }

    void link(unsigned int outNode, unsigned int outSocket,
              unsigned int inNode, unsigned int inSocket)
    {
         auto inNodeIt = m_nodeMap.find(inNode);
         auto outNodeIt = m_nodeMap.find(outNode);
         if(inNodeIt == m_nodeMap.end() || outNodeIt == m_nodeMap.end() ){
            ERRORMSG("In: " << inNode << " or " << outNode << " does not exist!")
         }
         LogicNode::linkTogether(outNodeIt->second,outSocket,inNodeIt->second,inSocket);
    }

    void execute() {
        for(auto n : m_nodes) {
            n->compute();
        }
    }

    virtual void setup() {

        if(m_outputNodes.size()== 0) {
            ERRORMSG("No output node specified")
        }

        // Solve Execution order, do a depth first search for all output nodes which determines an execution order by setting the priority
        m_inputReachable = false;
        for(auto & p : m_outputNodes){
            std::unordered_set<unsigned int> nodesCurrDepth;
            // start recursion
            solveExecutionOrder(p.second,nodesCurrDepth);
        }
        if(!m_inputReachable) {
            ERRORMSG("Your input node: " << m_inputNode->m_id << " cannot be reached by any output node!")
        }

        // Sort all nodes according to priority (asscending) (lowest is most important)
        std::sort(m_nodes.begin(), m_nodes.end(),
        [](LogicNode* const & a, LogicNode* const &b) {
            return a->getPriority() < b->getPriority();
        }
                 );

        // Invert priority such that lowest is now the highest and the first one!
        auto maxPrio = m_nodes.back()->getPriority();
        for(auto n : m_nodes) {
            n->setPriority( -n->getPriority() + maxPrio);
        }
        std::cout << "Execution order:" <<std::endl;
            for(auto n : m_nodes){
                std::cout << n->m_id <<": " << n->getPriority()<<std::endl;
            }
        std::cout << "===" <<std::endl;

    }

protected:

    template<bool input>
    void setInOutNode(unsigned int id){
        // Set output node
        auto it = m_nodeMap.find(id);
        if(it == m_nodeMap.end()) {
            ERRORMSG("No output node with id: " << id << " found " << std::endl)
        }
        if(input){
            m_inputNode = it->second;
        }else{
            m_outputNodes.emplace(id, it->second);
        }

    }

    /**
    * Breath first search: this function returns recursively the priority
    */
    unsigned int solveExecutionOrder( LogicNode* node,
                                      std::unordered_set<unsigned int> & nodesCurrDepth) {

        nodesCurrDepth.insert(node->m_id);

        if( node == m_inputNode) {
            m_inputReachable = true;
        }

        //visit all input sockets and their node!
        auto & inSockets = node->getInputs();
        for(auto * s : inSockets) {
            if(s->isLinked()) {

                auto * fsock = s->getFrom();
                ASSERTMSG(fsock,"linked but from ptr null");

                auto * adjNode = fsock->getParent();
                if(nodesCurrDepth.find(adjNode->m_id)!=nodesCurrDepth.end()) {
                    ERRORMSG("Your execution logic graph contains a cylce from node: " << node->m_id << " socket: " << s->m_id <<
                             " to node: " << adjNode->m_id << " socket: " << fsock->m_id);
                }

                unsigned int prioAdj = solveExecutionOrder(adjNode,nodesCurrDepth);

                // Set the current node to the priority of the below tree +1
                node->setPriority( std::max(  prioAdj+1, node->getPriority() ) );
            }
        }

        nodesCurrDepth.erase(node->m_id);

        return node->getPriority();

    }

    bool m_inputReachable = false;

    LogicNode * m_inputNode = nullptr;
    std::unordered_map<unsigned int, LogicNode *> m_outputNodes;

    std::vector<LogicNode*> m_nodes;
    std::unordered_map<unsigned int, LogicNode*> m_nodeMap;
};

#endif // ExecutionTreeInOut_hpp


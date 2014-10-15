#ifndef ExecutionTreeInOut_hpp
#define ExecutionTreeInOut_hpp

#include <algorithm>
#include <unordered_set>

#include <boost/mpl/vector.hpp>

#include "LogicNode.hpp"
#include "LogicSocket.hpp"


template<unsigned int NIN, unsigned int NOUT>
class DummyLogicNode : public LogicNode {
public:

    DEFINE_LAYOUT_CONFIG_TYPES;

    DummyLogicNode(unsigned int id) : LogicNode(id, NIN+NOUT) {
        for(unsigned int i=0; i<NIN; i++) {
            addISock(0.0);
        }
        for(unsigned int i=0; i<NOUT; i++) {
            addOSock(0.0);
        }
    }

    virtual ~DummyLogicNode() {}

    void compute() {
        double t = 0;
        for(unsigned int i=0; i<NIN; i++) {
            t += getSocketValue<double>(i);
        }
        std::cout << m_id << ": " << t << std::endl;
        for(unsigned int i=0; i<NOUT; i++) {
            setSocketValue(i+NIN,t+1);
        };
    }

};

//
//class SpecialNode : public LogicNode
//{
//public:
//
//    DEFINE_LAYOUT_CONFIG_TYPES;
//
//    using InputTypes  = boost::mpl::vector<double,int,std::string>;
//    using OutputTypes = boost::mpl::vector<double,int,std::string>;
//
//	DummyLogicNode(unsigned int id) : LogicNode(id, NIN+NOUT)
//	{
//		for(unsigned int i=0; i<NIN;i++){
//            addISock(1.0);
//		}
//		for(unsigned int i=0; i<NOUT;i++){
//            addOSock(1.0);
//		}
//	}
//
//	virtual ~DummyLogicNode() {}
//
//	void compute()
//	{
//	    double t = 0;
//	    for(unsigned int i=0; i<NIN;i++){
//            t += getSocketValue<double>(i);
//	    }
//		for(unsigned int i=0; i<NOUT;i++){
//            setSocketValue(i+NIN,t);
//	    };
//	}
//
//};



template<typename TInputNode>
class ExecutionTreeInOut {
private:
    using InputNodeType = TInputNode;
public:

    ExecutionTreeInOut() {
        // push input node
        m_inputNode = new InputNodeType(0);
        addNode(m_inputNode);
    }

    ~ExecutionTreeInOut() {
        for(auto & n : m_nodeMap) {
            delete n.second;
        }
    }

    LogicNode* getInputNode() {
        return m_inputNode;
    };

    void setOutputNode(unsigned int id) {
        // Set output node
        auto it = m_nodeMap.find(id);
        if(it == m_nodeMap.end()) {
            ERRORMSG("No output node with id: " << id << " found " << std::endl)
        }
        m_outputNode = it->second;
    }

    void addNode(LogicNode * node) {
        auto res = m_nodeMap.emplace(node->m_id, node);
        if(res.second) {
            m_nodes.push_back(node);
        } else {
            ERRORMSG("Node id: " << node->m_id << " already in map");
        }
    }

    void execute() {
        for(auto n : m_nodes) {
            n->compute();
        }
    }

    void setup() {

        if(!m_outputNode) {
            ERRORMSG("No output node specified")
        }


        m_inputReachable = false;
        std::unordered_set<unsigned int> nodesCurrDepth;
        // start recursion
        solveExecutionOrder(m_outputNode,nodesCurrDepth);

        if(!m_inputReachable) {
            ERRORMSG("Your input node: " << m_inputNode->m_id << " cannot be reached by the output node: " << m_outputNode->m_id)
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

private:
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
    LogicNode * m_outputNode = nullptr;

    std::vector<LogicNode*> m_nodes;
    std::unordered_map<unsigned int, LogicNode*> m_nodeMap;
};

#endif // ExecutionTreeInOut_hpp


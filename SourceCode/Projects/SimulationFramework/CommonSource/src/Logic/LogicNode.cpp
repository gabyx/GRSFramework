#include "LogicNode.hpp"
#include "LogicSocket.hpp"

LogicNode::LogicNode(unsigned int id, unsigned int nSockets = 2) :
        m_id(id), m_hasLinks(false), m_priority(0)
{
    m_sockets.assign(nSockets,nullptr);
}

LogicNode::~LogicNode()
{
	for(auto & socket : m_inputs){
        delete socket;
	}

	for(auto & socket : m_outputs){
        delete socket;
	}
}



LogicSocketBase* LogicNode::getISocket(unsigned int index)
{
    if (index >= 0 && index < m_inputs.size())
        return m_inputs.at(index);
    return nullptr;
}


LogicSocketBase* LogicNode::getOSocket(unsigned int index)
{
    if (index >= 0 && index < m_outputs.size())
        return m_outputs.at(index);
    return nullptr;
}

void LogicNode::linkTogether(LogicSocketBase * out, LogicSocketBase * in){
        //out->link(in);
        in->link(out); // tell the input where to get the value from!
}

void LogicNode::linkTogether(LogicNode * n1, unsigned int outSocketIdx,
                    LogicNode * n2, unsigned int inSocketIdx){
        //n1->getOSocket(outSocketIdx)->link(n2->getISocket(inSocketIdx));
        n2->getISocket(inSocketIdx)->link(n1->getOSocket(outSocketIdx));
}

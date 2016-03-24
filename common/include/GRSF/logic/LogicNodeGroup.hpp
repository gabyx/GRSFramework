#ifndef GRSF_logic_LogicNodeGroup_hpp
#define GRSF_logic_LogicNodeGroup_hpp

#include <vector>
#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/logic/LogicCommon.hpp"

class LogicSocketBase;
template<typename T> class LogicSocket;


class LogicNodeGroup : LogicNode
{
public:
    using SocketListType = LogicSocketCommon::SocketListType;
    using SocketIterator = LogicSocketCommon::SocketIterator;

    const unsigned int m_id;

public:
    LogicNodeGroup(unsigned int id);

	virtual ~LogicNodeGroup();

    /** some reset*/
    virtual void reset(){}

    /** the main compute function
        may be called many times
    */
    virtual void compute() {}

    virtual void addNode(LogicNode *)
protected:
    bool            m_hasLinks;
    SocketListType  m_inputs;
    SocketListType  m_outputs;
    unsigned int    m_priority;

    //SocketListType m_sockets;
};


#include "GRSF/logic/LogicSocket.hpp"


template<typename T>
LogicSocket<T>* LogicNode::getISocket(unsigned int idx)
{
    if (idx < m_inputs.size()){
        return m_inputs[idx]->castToType<T>();
    }
    return nullptr;
}
template<typename T>
LogicSocket<T>* LogicNode::getOSocket(unsigned int idx)
{
    if (idx < m_outputs.size()){
        return m_outputs[idx]->castToType<T>();
    }
    return nullptr;
}


template<typename T>
T LogicNode::getISocketValue(unsigned int idx)
{
    return m_inputs[idx]->castToType<T>()->getValue();
}
template<typename T>
T LogicNode::getOSocketValue(unsigned int idx)
{
    return m_outputs[idx]->castToType<T>()->getValue();
}

template<typename T>
T & LogicNode::getISocketRefValue(unsigned int idx)
{
    return m_inputs[idx]->castToType<T>()->getRefValue();
}
template<typename T>
T & LogicNode::getOSocketRefValue(unsigned int idx)
{
    return m_outputs[idx]->castToType<T>()->getRefValue();
}



template<typename T, typename TIn>
void LogicNode::setISocketValue(unsigned int idx, const TIn & data)
{
    m_inputs[idx]->castToType<T>()->setValue(data);
}
template<typename T, typename TIn>
void LogicNode::setOSocketValue(unsigned int idx, const TIn & data)
{
    m_outputs[idx]->castToType<T>()->setValue(data);
}


#endif //LogicNode_hpp


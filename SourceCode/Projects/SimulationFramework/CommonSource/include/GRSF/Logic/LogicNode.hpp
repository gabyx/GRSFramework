#ifndef GRSF_Logic_LogicNode_hpp
#define GRSF_Logic_LogicNode_hpp

#include <vector>
#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Logic/LogicCommon.hpp"

class LogicSocketBase;
template<typename T> class LogicSocket;


class LogicNode
{
public:
    using SocketListType = LogicSocketCommon::SocketListType;
    using SocketIterator = LogicSocketCommon::SocketIterator;

    const unsigned int m_id;

public:
    LogicNode(unsigned int id);

	virtual ~LogicNode();

    // do the update logic
    virtual void compute() {}

    // do first run initialization
    virtual void reset(){}

    LogicSocketBase*   getISocket(unsigned int index);
    LogicSocketBase*   getOSocket(unsigned int index);

    inline bool          hasLinks() const {return m_hasLinks;}
    inline void          setLinked(void){m_hasLinks = true;}
    inline void          setPriority(unsigned int v){m_priority = v;}
    inline unsigned int  getPriority(void) const {return m_priority;}


    SocketListType& getInputs()  {return m_inputs;}
    SocketListType& getOutputs() {return m_outputs;}


	template<typename T>
	void addISock(const T & defaultValue)
	{
	    unsigned int idx = m_inputs.size() + m_outputs.size();
	    auto * t = new LogicSocket<T>(this, true, defaultValue, idx);
		m_inputs.push_back(t);

//		ASSERTMSG(idx < m_sockets.size(),"Node: " << m_id << " idx: " << idx);
//		m_sockets[idx] = t;

	}

	template<typename T>
	void addOSock(const T & defaultValue)
	{
	    unsigned int idx = m_inputs.size() + m_outputs.size();
		auto * t = new LogicSocket<T>(this, false, defaultValue, idx);
		m_outputs.push_back(t);

//		ASSERTMSG(idx < m_sockets.size(),"Node: " << m_id << " idx: " << idx);
//		m_sockets[idx] = t;
	}

	template<typename T> LogicSocket<T>* getISocket(unsigned int idx);
    template<typename T> LogicSocket<T>* getOSocket(unsigned int idx);

	template<typename T> T getISocketValue(unsigned int idx);
	template<typename T> T& getISocketRefValue(unsigned int idx);
    template<typename T> T getOSocketValue(unsigned int idx);
	template<typename T> T& getOSocketRefValue(unsigned int idx);

	template<typename T, typename TIn> void setISocketValue(unsigned int idx, const TIn & data);
    template<typename T, typename TIn> void setOSocketValue(unsigned int idx, const TIn & data);

    /**
    * Links together an output with an input. Get the data from output from the input
    */
    static void makeGetLink(LogicNode * outN, unsigned int outS,
                            LogicNode * inN,  unsigned int inS);
    /**
    * Links together an output with an input. Write the data from output to the input
    */
    static void makeWriteLink(LogicNode * outN, unsigned int outS,
                              LogicNode * inN,  unsigned int inS);

protected:
    bool            m_hasLinks;
    SocketListType  m_inputs;
    SocketListType  m_outputs;
    unsigned int    m_priority;

    //SocketListType m_sockets;
};


#include "GRSF/Logic/LogicSocket.hpp"


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

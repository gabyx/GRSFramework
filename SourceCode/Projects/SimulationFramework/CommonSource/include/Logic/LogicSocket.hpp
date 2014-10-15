#ifndef LogicSocket_hpp
#define LogicSocket_hpp

#include <boost/mpl/at.hpp>
#include <boost/mpl/find.hpp>
#include <boost/mpl/vector.hpp>

#include "StaticAssert.hpp"
#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

#include "EnumClassHelper.hpp"

#include "LogicCommon.hpp"

class LogicNode;
template<typename T> class LogicSocket;

class LogicSocketBase {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using SocketListType = LogicSocketCommon::SocketListType;
    using SocketIterator = LogicSocketCommon::SocketIterator;

    const unsigned int m_id = 0;
    const unsigned int m_type; ///< The index in the mpl sequence, which type this is!

    using TypeSeq = boost::mpl::vector<double,
                                       float,
                                       char,
                                       short,
                                       int,
                                       unsigned int,
                                       unsigned long,
                                       unsigned long long,
                                       std::string,
                                       Vector3,
                                       Quaternion,
                                       VectorQBody,
                                       VectorUBody>;

    /**
    * Cast to a LogicSocket<T> *,
    */
    template<typename T>
    inline
    LogicSocket<T> * castToType()
    {
        typedef typename boost::mpl::find<TypeSeq,T>::type iter;
        ASSERTMSG( this->m_type == iter::pos::value, " Types of sockets do not coincide: " << this->m_type << " and " << iter::pos::value);
        return static_cast< LogicSocket<T> * >(this);
    }

    LogicSocketBase(unsigned int type)
        : m_type(type), m_id(0), m_isInput(true), m_from(nullptr), m_connected(false), m_parent(nullptr) {
    }

    LogicSocketBase(unsigned int type, LogicNode* par, bool isInput, unsigned int id)
        : m_type(type), m_id(id),  m_isInput(isInput), m_from(nullptr), m_connected(false), m_parent(par) {
    }

    virtual ~LogicSocketBase() {};

    void link(LogicSocketBase *fsock);

    // owner node
    inline LogicNode* getParent() const {
        return m_parent;
    }

    inline bool isLinked() const {
        return m_from != nullptr;
    }

    inline bool isConnected() const {
        return m_connected;
    }

    inline LogicSocketBase* getFrom() const {
        return m_from;
    }

protected:

    bool m_isInput;

    // from socket to 'this' (used to link an input socket with an output socket)
    // Only one makes sense
    LogicSocketBase* m_from;

    // from 'this' to sockets (used to link an output socket with one or more than one input socket)
    SocketListType m_to;

private:

    bool m_connected;
    // owner node
    LogicNode* m_parent;
};



template<typename T>
class LogicSocket : public LogicSocketBase {
private:
    typedef typename boost::mpl::find<TypeSeq,T>::type iter;
    typedef boost::mpl::end<TypeSeq>::type endIter;
public:

    /** This assert fails if the type T of the LogicSocket is not properly added to the mpl list in LogicSocketBase*/
    STATIC_ASSERT(iter::pos::value != endIter::pos::value )

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LogicSocket()
        : LogicSocketBase(iter::pos::value) {
    }

    LogicSocket(LogicNode* par, bool isInput, T defaultValue, unsigned int id)
        : LogicSocketBase(iter::pos::value,par, isInput, id), m_data(defaultValue) {
    }

    void setValue(const T& value);
    T getValue() const;

    T& getRefValue();

private:

    T m_data; ///< Default value!
};




#include "LogicNode.hpp"

template<typename T>
void LogicSocket<T>::setValue(const T& value) {
    if(!m_isInput) {
        // set all "to" sockets
        for(auto & s : m_to) {
            ASSERTMSG(s->m_type == m_type, "Types of node have to match: type "
                      << m_type << "from id: " << s->getParent()->m_id
                      << "to type " << s->m_type <<" of id: " << s->getParent()->m_id )
            LogicSocket<T>* sock = static_cast< LogicSocket<T>* > (s);
            sock->setValue(value);
        }
    }

    m_data = value;
};

template<typename T>
T LogicSocket<T>::getValue() const {
    if(m_from) {

        LogicSocket<T>* sock = static_cast<LogicSocket<T>*>(m_from);

        ASSERTMSG(sock , "Types have to match");

        return sock->getValue();
    }

    return m_data;
};

template<typename T>
T & LogicSocket<T>::getRefValue() {
    if(m_from) {
        LogicSocket<T>* sock = dynamic_cast<LogicSocket<T>*>(m_from);

        ASSERTMSG(sock , "Types have to match");

        return sock->getRefValue();
    }

    return m_data;
}

template<typename T>
LogicSocket<T>* getSocket(LogicSocketBase* pSock) {
    return static_cast<LogicSocket<T>*>(pSock);
}

#define ADD_ISOCK( name, value )      \
	addISock < _ISOCKET_TYPE_##name > ( value );	\


#define ADD_OSOCK(name, value)      \
	addOSock< _OSOCKET_TYPE_##name > (  value );	\


#define SET_ISOCKET_VALUE( name, value )	\
	setSocketValue< _ISOCKET_TYPE_##name > ( (Inputs::name), (value) )

#define SET_OSOCKET_VALUE( name, value )	\
	setSocketValue< _OSOCKET_TYPE_##name > ( (Outputs::name), (value) )



#define GET_ISOCKET_VALUE( name )      \
	getSocketValue< _SOCKET_TYPE_##name > ( (Inputs::name) )

#define GET_OSOCKET_VALUE( name )      \
	getSocketValue< _SOCKET_TYPE_##name > ( (Outputs::name) )


#define GET_ISOCKET_REF_VALUE( name )      \
	getSocketRefValue< _SOCKET_TYPE_##name > ( (Inputs::name) )

#define GET_OSOCKET_REF_VALUE( name )      \
	getSocketRefValue< _SOCKET_TYPE_##name > ( (Outputs::name) )


#define GET_ISOCKET( name )      \
	getSocket< _ISOCKET_TYPE_##name > ( (Inputs::name) )

#define GET_OSOCKET( name )      \
	getSocket< _OSOCKET_TYPE_##name > ( (Outputs::name) )


#define DECLARE_ISOCKET_TYPE( name, type)	\
	typedef type _ISOCKET_TYPE_##name;	\
	inline LogicSocket< type >* get##name() { return getSocket< type > ( (Inputs::name) ); }

#define DECLARE_OSOCKET_TYPE( name, type)	\
	typedef type _OSOCKET_TYPE_##name;	\
	inline LogicSocket< type >* get##name() { return getSocket< type > ( (Outputs::name) ); }


#endif//_LogicSocket_h_

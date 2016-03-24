// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_LogicSocket_hpp
#define GRSF_logic_LogicSocket_hpp

#include <boost/mpl/at.hpp>
#include <boost/mpl/find.hpp>
#include <boost/mpl/vector.hpp>

#include "GRSF/common/StaticAssert.hpp"
#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/EnumClassHelper.hpp"

#include "GRSF/logic/LogicCommon.hpp"


class LogicNode;
template<typename T> class LogicSocket;

class LogicSocketBase {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using SocketListType = LogicSocketCommon::SocketListType;
    using SocketIterator = LogicSocketCommon::SocketIterator;

    const unsigned int m_id = 0;
    const unsigned int m_type; ///< The index in the mpl sequence, which type this is!

    /** Using an external logic type list */
    using TypeSeq = LogicTypes::TypeSeq;


    /**
    * Cast to a LogicSocket<T> *,
    */
    template<typename T>
    inline
    LogicSocket<T> * castToType()
    {
        typedef typename boost::mpl::find<TypeSeq,T>::type iter;
        ASSERTMSG( this->m_type == iter::pos::value,
                  " Types of sockets do not coincide: " <<
                  LogicTypes::getTypeName(this->m_type) << " and " << LogicTypes::getTypeName(iter::pos::value) );
        return static_cast< LogicSocket<T> * >(this);
    }

    template<typename TVisitor>
    void applyVisitor(TVisitor && visitor){
        // implement switch statement in LogicTypes
        LOGICSOCKET_APPLY_VISITOR_SWITCH;
    }


    LogicSocketBase(unsigned int type)
        : m_id(0), m_type(type), m_isInput(true), m_from(nullptr), m_connected(false), m_parent(nullptr) {
    }

    LogicSocketBase(unsigned int type, LogicNode* par, bool isInput, unsigned int id)
        :  m_id(id), m_type(type),  m_isInput(isInput), m_from(nullptr), m_connected(false), m_parent(par) {
    }

    virtual ~LogicSocketBase() {};

    void link(LogicSocketBase *fsock);

    // owner node
    inline LogicNode* getParent() const {
        return m_parent;
    }

    /** Linked means if and input is connected */
    inline bool isLinked() const {
        return m_from != nullptr;
    }

    /** Connected means if output is connected somewhere or if input is connected */
    inline bool isConnected() const {
        return m_connected;
    }

    inline LogicSocketBase* getFrom() const {
        return m_from;
    }

protected:

    bool m_isInput;

    /** from socket to 'this' (used to link an input socket with an output socket)
    *  incoming edges: only one makes sense
    *  only valid for input sockets
    */
    LogicSocketBase* m_from;

    /** from 'this' to sockets (used to link an output socket with one or more than one input socket)
    *   outgoing edges
    *   only valid for output sockets
    */
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
        : LogicSocketBase(iter::pos::value, par, isInput, id), m_data(defaultValue) {
    }

    template<typename TIn>
    void setValue(const TIn & value);

    void distributeValue(); ///< Only for output socket

    T getValue() const;

    T& getValueRef();

private:

    T m_data; ///< Default value! or the output value if output socket
};




#include "GRSF/logic/LogicNode.hpp"


template<typename T>
void LogicSocket<T>::distributeValue() {

   if(!m_isInput) {
        // if output node
        // set all "to" sockets ( these are write links, because we write the value to another input socket)
        for(auto & s : m_to) {
            ASSERTMSG(s->m_type == m_type, "Types of node have to match: type "
                      << m_type << "from id: " << s->getParent()->m_id
                      << "to type " << s->m_type <<" of id: " << s->getParent()->m_id )
            LogicSocket<T>* sock = static_cast< LogicSocket<T>* > (s);
            sock->setValue(m_data);
        }
    }

}

template<typename T>
template<typename TIn>
void LogicSocket<T>::setValue(const TIn& value) {
    // set internal value for input node, however if output node we also set the value ( might be needed
    m_data = value;
    distributeValue();
};

template<typename T>
T LogicSocket<T>::getValue() const {

    // if we have a "from" node (only for input sockets), we get the value from this sockets
    if(m_from) {
        return m_from->castToType<T>()->getValue();
    }
    return m_data;
};

template<typename T>
T & LogicSocket<T>::getValueRef() {

    // if we have a from"
    if(m_from) {
        return m_from->castToType<T>()->getValueRef();
    }

    return m_data;
}

#define ADD_ISOCK( name, value )      \
    ASSERTMSG(Inputs::name == this->m_inputs.size(), " Wrong order for Input: " << Inputs::name) \
	addISock < IType##name > ( value );	\


#define ADD_OSOCK(name, value)      \
    ASSERTMSG(Outputs::name == this->m_outputs.size(), " Wrong order for Output: " << Outputs::name) \
	addOSock< OType##name > (  value );	\

#define REMOVEPTR_REMOVEREF( _ptr_ ) \
    std::remove_pointer<typename std::remove_reference< decltype(_ptr_) >::type >::type

/** VALUE SETTER MACROS */
#define SET_ISOCKET_VALUE_PTR(ptr, name, value )	\
    ptr->setISocketValue< typename REMOVEPTR_REMOVEREF(ptr)::IType##name > ( ( REMOVEPTR_REMOVEREF(ptr)::Inputs::name), value )

#define SET_OSOCKET_VALUE_PTR(ptr, name, value )	\
    ptr->setOSocketValue< typename REMOVEPTR_REMOVEREF(ptr)::OType##name > ( ( REMOVEPTR_REMOVEREF(ptr)::Outputs::name), value )

#define DISTRIBUTE_OSOCKET_VALUE_PTR(ptr, name)	\
    ptr->distributeOSocketValue< typename REMOVEPTR_REMOVEREF(ptr)::OType##name > ( ( REMOVEPTR_REMOVEREF(ptr)::Outputs::name) )

#define SET_ISOCKET_VALUE( name, value )	\
	SET_ISOCKET_VALUE_PTR(this,name,value)

#define SET_OSOCKET_VALUE( name, value )	\
	SET_OSOCKET_VALUE_PTR(this,name,value)

#define DISTRIBUTE_OSOCKET_VALUE( name )	\
	DISTRIBUTE_OSOCKET_VALUE_PTR(this,name)

/** VALUE GETTER MACROS */
#define GET_ISOCKET_VALUE_PTR( ptr, name )      \
	ptr->getISocketValue< typename REMOVEPTR_REMOVEREF(ptr)::IType##name > ( (REMOVEPTR_REMOVEREF(ptr)::Inputs::name) )

#define GET_OSOCKET_VALUE_PTR( ptr, name )      \
	ptr->getOSocketValue< typename REMOVEPTR_REMOVEREF(ptr)::OType##name > ( (REMOVEPTR_REMOVEREF(ptr)::Outputs::name) )

#define GET_ISOCKET_VALUE( name )      \
	GET_ISOCKET_VALUE_PTR(this,name)

#define GET_OSOCKET_VALUE( name )      \
	GET_OSOCKET_VALUE_PTR(this,name)

/** VALUE GETTER REFERENCE MACROS */
#define GET_ISOCKET_REF_VALUE_PTR(ptr, name )      \
	getISocketRefValue< IType##name > ( (Inputs::name) )

#define GET_OSOCKET_REF_VALUE_PTR(ptr, name )      \
	getOSocketRefValue< OType##name > ( (Outputs::name) )

#define GET_ISOCKET_REF_VALUE( name )      \
	GET_ISOCKET_REF_VALUE_PTR(this,name)

#define GET_OSOCKET_REF_VALUE( name )      \
	GET_OSOCKET_REF_VALUE_PTR(this,name)

/** SOCKET GETTER REFERENCE MACROS */
#define GET_ISOCKET_PTR( ptr, name )      \
	getISocket< IType##name > ( (Inputs::name) )

#define GET_OSOCKET_PTR( ptr, name )      \
	getOSocket< OType##name > ( (Outputs::name) )

#define GET_ISOCKET( name )      \
	GET_ISOCKET_PTR(this,name)

#define GET_OSOCKET( name )      \
	GET_OSOCKET_PTR(this,name)


/** SOCKET DECLARATION MACROS */
#define DECLARE_ISOCKET_TYPE( name, type)	\
	typedef type IType##name;	\
	inline LogicSocket< type >* getIn_##name() { \
	    return getISocket< type >( (Inputs::name) ); \
    }

#define DECLARE_OSOCKET_TYPE( name, type)	\
	typedef type OType##name;	\
	inline LogicSocket< type >* getOut_##name() { \
        return getOSocket< type >( (Outputs::name) ); \
	}


#endif//_LogicSocket_h_

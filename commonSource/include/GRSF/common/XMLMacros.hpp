
#ifndef GRSF_Common_XMLMacros_hpp
#define GRSF_Common_XMLMacros_hpp

#include "GRSF/Common/AssertionDebug.hpp"

#define CHECK_XMLNODE( _node_ , _nodename_ ) \
    if( ! _node_ ){ \
        THROWEXCEPTION("XML Node: " << _nodename_ << " does not exist!");  \
    }
#define CHECK_XMLATTRIBUTE( _node_ , _nodename_ ) \
    if( ! _node_ ){ \
        THROWEXCEPTION("XML Attribute: " << _nodename_ << " does not exist!");  \
    }

#define GET_XMLCHILDNODE_CHECK( _childnode_ , _childname_ , _node_ ) \
    _childnode_ = _node_.child( _childname_ ); \
    CHECK_XMLNODE( _childnode_ , _childname_)

#define GET_XMLATTRIBUTE_CHECK( _att_ , _attname_ , _node_ ) \
    _att_ = _node_.attribute( _attname_ ); \
    CHECK_XMLATTRIBUTE( _att_ , _attname_ )

#endif // XMLMacros_hpp



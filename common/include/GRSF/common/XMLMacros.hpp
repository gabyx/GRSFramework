// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_XMLMacros_hpp
#define GRSF_common_XMLMacros_hpp

#include "GRSF/common/Asserts.hpp"

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



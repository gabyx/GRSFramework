// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_general_LogicParserBaseTraits_hpp
#define GRSF_general_LogicParserBaseTraits_hpp

#include "GRSF/common/SimpleLogger.hpp"
#include <pugixml.hpp>


template<typename TSceneParser, typename TDataStorage>
struct LogicParserBaseTraits {

    using DataStorageType = TDataStorage;

    using ParserType = TSceneParser;
    using LogType = Logging::Log;

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;
};


#include "GRSF/general/LogicParserBaseTraitsMacro.hpp"


#endif

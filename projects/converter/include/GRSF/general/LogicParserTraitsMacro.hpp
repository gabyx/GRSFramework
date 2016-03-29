// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_general_LogicParserTraitsMacro_hpp
#define GRSF_general_LogicParserTraitsMacro_hpp

#include "GRSF/general/LogicParserBaseTraitsMacro.hpp"

#define DEFINE_LOGICPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_LOGICPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using LogicModuleType         = typename TParserTraits::LogicModuleType;\

#endif

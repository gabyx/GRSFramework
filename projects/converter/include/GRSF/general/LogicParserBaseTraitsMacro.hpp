// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_general_LogicParserBaseTraitsMacro_hpp
#define GRSF_general_LogicParserBaseTraitsMacro_hpp

#define DEFINE_LOGICPARSER_BASE_TYPE_TRAITS(TParserTraits)            \
    using ParserType       = typename TParserTraits::ParserType;      \
    using DataStorageType  = typename TParserTraits::DataStorageType; \
    using LogType          = typename TParserTraits::LogType;         \
    using XMLNodeType      = typename TParserTraits::XMLNodeType;     \
    using XMLNodeItType    = typename TParserTraits::XMLNodeItType;   \
    using XMLAttributeType = typename TParserTraits::XMLAttributeType;

#endif

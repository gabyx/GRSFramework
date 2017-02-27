// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_renderer_RenderLogicParserTraits_hpp
#define GRSF_converters_renderer_RenderLogicParserTraits_hpp

#include "GRSF/general/LogicParserBaseTraits.hpp"

#include "GRSF/converters/renderer/RenderLogicParserModules.hpp"

/** The traits for a standart RenderLogicParser class*/
template <typename TSceneParser, typename TDataStorage>
struct RenderLogicParserTraits : LogicParserBaseTraits<TSceneParser, TDataStorage>
{
    // Module typedefs
    using MaterialsModuleType = typename RenderLogicParserModules::MaterialsModule<RenderLogicParserTraits>;
    using LogicModuleType     = typename RenderLogicParserModules::LogicModule<RenderLogicParserTraits>;

    using TupleModules = std::tuple<std::unique_ptr<MaterialsModuleType>, std::unique_ptr<LogicModuleType>>;

    template <unsigned int N>
    using getModuleType = typename std::tuple_element<N, TupleModules>::type::element_type;
};

#include "GRSF/converters/renderer/RenderLogicParserTraitsMacro.hpp"

#endif

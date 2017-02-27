// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_systems_SceneParserMPI_hpp
#define GRSF_systems_SceneParserMPI_hpp

#include "GRSF/systems/SceneParser.hpp"

#include "GRSF/systems/SceneParserModulesMPI.hpp"

/** These module types are defined when there is no derivation from scene parser */
template <typename TSceneParser, typename TDynamicsSystem>
struct SceneParserMPITraits : public SceneParserBaseTraits<TSceneParser, TDynamicsSystem>
{
    using SettingsModuleType       = ParserModules::SettingsModuleMPI<SceneParserMPITraits>;
    using ExternalForcesModuleType = ParserModules::ExternalForcesModule<SceneParserMPITraits>;
    using ContactParamModuleType   = ParserModules::ContactParamModule<SceneParserMPITraits>;
    using InitStatesModuleType     = ParserModules::InitStatesModule<SceneParserMPITraits>;

    using BodyModuleType     = ParserModules::BodyModule<SceneParserMPITraits>;
    using GeometryModuleType = ParserModules::GeometryModule<SceneParserMPITraits>;

    using VisModuleType = ParserModules::VisModuleDummy<SceneParserMPITraits>;

    using MPIModuleType = ParserModules::MPIModule<SceneParserMPITraits>;
};

template <typename TDynamicsSystem, template <typename P, typename D> class TParserTraits = SceneParserMPITraits>
class SceneParserMPI : public SceneParser<TDynamicsSystem, TParserTraits, SceneParserMPI<TDynamicsSystem>>
{
private:
    using BaseType = SceneParser<TDynamicsSystem, TParserTraits, SceneParserMPI<TDynamicsSystem>>;

public:
    using DynamicsSystemType = TDynamicsSystem;

public:
    template <typename ModuleGeneratorType>
    SceneParserMPI(ModuleGeneratorType& moduleGen, Logging::Log* log, const boost::filesystem::path& mediaDir)
        : BaseType(moduleGen, log, mediaDir)
    {
    }
};

#endif

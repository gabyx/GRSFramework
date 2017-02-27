// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_DynamicsSystemMPI_hpp
#define GRSF_dynamics_general_DynamicsSystemMPI_hpp

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/systems/SceneParserModules.hpp"
#include "GRSF/systems/SceneParserModulesMPI.hpp"

#include "GRSF/dynamics/general/DynamicsSystemBase.hpp"

#include "GRSF/dynamics/general/MPITopologyBuilderSettings.hpp"

class DynamicsSystemMPI : public DynamicsSystemBase
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DEFINE_DYNAMICSYSTEM_BASE_TYPES

    DynamicsSystemMPI();
    ~DynamicsSystemMPI();

    using TopologyBuilderSettingsType = MPILayer::TopologyBuilderSettings;

    RigidBodySimContainerType m_remoteSimBodies;  // all remote bodies

    void deleteSimBodies();  ///< deletes all local and remote bodies, static bodies are not considered!

    inline void addSimBodyPtr(RigidBodyType* ptr)
    {
        m_simBodies.addBody(ptr);
    }
    inline void addBodyPtr(RigidBodyType* ptr)
    {
        m_staticBodies.addBody(ptr);
    }

    const TopologyBuilderSettingsType& getSettingsTopoBuilder() const;

    protected:
    TopologyBuilderSettingsType m_settingsTopologyBuilder;

    public:
    struct ParserModulesCreator1
    {
        ParserModulesCreator1(DynamicsSystemMPI* p) : m_p(p)
        {
        }
        DynamicsSystemMPI* m_p;

        template <typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser, TDynamicsSystem>
        {
            // Module typedefs
            using SettingsModuleType       = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType   = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType     = ParserModules::InitStatesModule<SceneParserTraits>;

            // Default templates
            //           bool parseAllIfRangeEmpty = true,
            //           bool parseAllBodiesNonSelGroup = true ,
            //           bool parseSimBodies = true,
            //           bool parseStaticBodies = true ,
            //           bool allocateSimBodies = true,
            //           bool allocateStaticBodies = true,
            //           bool parseInitialCondition = true
            using BodyMStaticOptions =
                ParserModules::BodyModuleStaticOptions<true, true, false, true, false, true, true>;
            using BodyModuleType = ParserModules::BodyModule<SceneParserTraits, BodyMStaticOptions>;

            using GeomMStaticOptions = ParserModules::GeometryModuleStaticOptions<>;
            using GeometryModuleType = ParserModules::GeometryModule<SceneParserTraits, GeomMStaticOptions>;

            using VisModuleType = ParserModules::VisModuleDummy<SceneParserTraits>;
            using MPIModuleType = ParserModules::MPIModule<SceneParserTraits>;
        };

        template <typename TParser>
        std::tuple<std::unique_ptr<typename TParser::SettingsModuleType>,
                   std::unique_ptr<typename TParser::ExternalForcesModuleType>,
                   std::unique_ptr<typename TParser::ContactParamModuleType>,
                   std::unique_ptr<typename TParser::InitStatesModuleType>,
                   std::unique_ptr<typename TParser::BodyModuleType>,
                   std::unique_ptr<typename TParser::GeometryModuleType>,
                   std::unique_ptr<typename TParser::VisModuleType>,
                   std::unique_ptr<typename TParser::MPIModuleType>>
        createParserModules(TParser* p)
        {
            using SettingsModuleType       = typename TParser::SettingsModuleType;
            using ContactParamModuleType   = typename TParser::ContactParamModuleType;
            using GeometryModuleType       = typename TParser::GeometryModuleType;
            using InitStatesModuleType     = typename TParser::InitStatesModuleType;
            using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType;
            using BodyModuleType           = typename TParser::BodyModuleType;
            using VisModuleType            = typename TParser::VisModuleType;
            using MPIModuleType            = typename TParser::MPIModuleType;

            auto sett = std::unique_ptr<SettingsModuleType>(new SettingsModuleType(
                p, &m_p->m_settingsRecorder, &m_p->m_settingsTimestepper, &m_p->m_settingsInclusionSolver));

            auto geom = std::unique_ptr<GeometryModuleType>(new GeometryModuleType(p, &m_p->m_globalGeometries));

            auto is = std::unique_ptr<InitStatesModuleType>(
                new InitStatesModuleType(p, &m_p->m_bodiesInitStates, &m_p->m_settingsTimestepper));
            auto vis = std::unique_ptr<VisModuleType>(nullptr);  // no visualization needed
            auto bm  = std::unique_ptr<BodyModuleType>(
                new BodyModuleType(p, geom.get(), is.get(), vis.get(), &m_p->m_simBodies, &m_p->m_staticBodies));
            auto es =
                std::unique_ptr<ExternalForcesModuleType>(new ExternalForcesModuleType(p, &m_p->m_externalForces));
            auto con =
                std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p, &m_p->m_ContactParameterMap));

            auto mpi = std::unique_ptr<MPIModuleType>(new MPIModuleType(p, bm.get(), &m_p->m_settingsTopologyBuilder));

            return std::make_tuple(std::move(sett),
                                   std::move(es),
                                   std::move(con),
                                   std::move(is),
                                   std::move(bm),
                                   std::move(geom),
                                   std::move(vis),
                                   std::move(mpi));
        }
    };

    struct ParserModulesCreator2
    {
        ParserModulesCreator2(DynamicsSystemMPI* p) : m_p(p)
        {
        }
        DynamicsSystemMPI* m_p;

        template <typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser, TDynamicsSystem>
        {
            // Module typedefs
            using SettingsModuleType       = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType   = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType     = ParserModules::InitStatesModule<SceneParserTraits>;

            // Default templates
            //           bool parseAllIfRangeEmpty = true,
            //           bool parseAllBodiesNonSelGroup = true ,
            //           bool parseSimBodies = true,
            //           bool parseStaticBodies = true ,
            //           bool allocateSimBodies = true,
            //           bool allocateStaticBodies = true,
            //           bool parseInitialCondition = true

            //        m_parseAllBodiesNonSelGroup = true;
            //        m_parseSimBodies = true; m_allocateSimBodies = true;
            //        m_parseStaticBodies = false; m_allocateStaticBodies = false;
            //        optBody.m_parseInitialCondition = false;
            //        m_parseAllIfRangeEmpty = false;
            using BodyMStaticOptions =
                ParserModules::BodyModuleStaticOptions<false, true, true, false, true, false, false>;
            using BodyModuleType = ParserModules::BodyModule<SceneParserTraits, BodyMStaticOptions>;

            using GeomMStaticOptions = ParserModules::GeometryModuleStaticOptions<>;
            using GeometryModuleType = ParserModules::GeometryModule<SceneParserTraits, GeomMStaticOptions>;

            using VisModuleType = ParserModules::VisModuleDummy<SceneParserTraits>;
            using MPIModuleType = ParserModules::MPIModule<SceneParserTraits>;
        };

        template <typename TParser>
        std::tuple<std::unique_ptr<typename TParser::SettingsModuleType>,
                   std::unique_ptr<typename TParser::ExternalForcesModuleType>,
                   std::unique_ptr<typename TParser::ContactParamModuleType>,
                   std::unique_ptr<typename TParser::InitStatesModuleType>,
                   std::unique_ptr<typename TParser::BodyModuleType>,
                   std::unique_ptr<typename TParser::GeometryModuleType>,
                   std::unique_ptr<typename TParser::VisModuleType>,
                   std::unique_ptr<typename TParser::MPIModuleType>>
        createParserModules(TParser* p)
        {
            using SettingsModuleType       = typename TParser::SettingsModuleType;
            using ContactParamModuleType   = typename TParser::ContactParamModuleType;
            using GeometryModuleType       = typename TParser::GeometryModuleType;
            using InitStatesModuleType     = typename TParser::InitStatesModuleType;
            using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType;
            using BodyModuleType           = typename TParser::BodyModuleType;
            using VisModuleType            = typename TParser::VisModuleType;
            using MPIModuleType            = typename TParser::MPIModuleType;

            auto sett = std::unique_ptr<SettingsModuleType>(new SettingsModuleType(
                p, &m_p->m_settingsRecorder, &m_p->m_settingsTimestepper, &m_p->m_settingsInclusionSolver));

            auto geom = std::unique_ptr<GeometryModuleType>(new GeometryModuleType(p, &m_p->m_globalGeometries));

            auto is = std::unique_ptr<InitStatesModuleType>(
                new InitStatesModuleType(p, &m_p->m_bodiesInitStates, &m_p->m_settingsTimestepper));
            auto vis = std::unique_ptr<VisModuleType>(nullptr);  // no visualization needed
            auto bm  = std::unique_ptr<BodyModuleType>(
                new BodyModuleType(p, geom.get(), is.get(), vis.get(), &m_p->m_simBodies, &m_p->m_staticBodies));
            auto es =
                std::unique_ptr<ExternalForcesModuleType>(new ExternalForcesModuleType(p, &m_p->m_externalForces));
            auto con =
                std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p, &m_p->m_ContactParameterMap));

            auto mpi = std::unique_ptr<MPIModuleType>(new MPIModuleType(p, bm.get(), &m_p->m_settingsTopologyBuilder));

            return std::make_tuple(std::move(sett),
                                   std::move(es),
                                   std::move(con),
                                   std::move(is),
                                   std::move(bm),
                                   std::move(geom),
                                   std::move(vis),
                                   std::move(mpi));
        }
    };
};

#endif

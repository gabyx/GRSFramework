// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_converters_renderer_RenderLogicParserGenerators_hpp
#define GRSF_converters_renderer_RenderLogicParserGenerators_hpp


#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"

#include <memory>

// This file should only be included in compilation units!

class RenderData;


#include "GRSF/systems/SceneParserModules.hpp"

namespace RenderLogicParserGenerators {
    struct SceneParserGen {
        SceneParserGen( RenderData * p): m_p(p) {}
        RenderData * m_p;

        template<typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser,TDynamicsSystem> {
            // Module typedefs
            using SettingsModuleType         = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

            using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<true,true,true,true,false,false,false>;
            using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits, BodyMStaticOptions > ;

            using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<false,true,false,true>;
            using GeometryModuleType         = ParserModules::GeometryModule<SceneParserTraits,GeomMStaticOptions>;

            using VisModuleType              = ParserModules::VisModuleConverter<SceneParserTraits>;
            using MPIModuleType              = ParserModules::MPIModuleDummy<SceneParserTraits>;
        };


        template<typename TParser>
        std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
            std::unique_ptr<typename TParser::ExternalForcesModuleType >,
            std::unique_ptr<typename TParser::ContactParamModuleType>,
            std::unique_ptr<typename TParser::InitStatesModuleType >,
            std::unique_ptr<typename TParser::BodyModuleType >,
            std::unique_ptr<typename TParser::GeometryModuleType >,
            std::unique_ptr<typename TParser::VisModuleType>,
            std::unique_ptr<typename TParser::MPIModuleType>
            >
        createParserModules(TParser * p) {

            using SettingsModuleType       = typename TParser::SettingsModuleType ;
            using ContactParamModuleType   = typename TParser::ContactParamModuleType;
            using GeometryModuleType       = typename TParser::GeometryModuleType ;
            using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
            using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
            using BodyModuleType           = typename TParser::BodyModuleType ;
            using VisModuleType            = typename TParser::VisModuleType ;
            using MPIModuleType            = typename TParser::MPIModuleType ;

            auto sett = std::unique_ptr<SettingsModuleType >(nullptr);
            auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &m_p->m_globalGeometries, nullptr, &m_p->m_geometryMap) );
            auto is  = std::unique_ptr<InitStatesModuleType >(nullptr);
            auto vis = std::unique_ptr<VisModuleType>(new VisModuleType(p, &m_p->m_scales, &m_p->m_visMeshs)); // no visualization needed
            auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p, geom.get(), nullptr, vis.get() , nullptr, nullptr )) ;
            auto es  = std::unique_ptr<ExternalForcesModuleType >(nullptr);
            auto con = std::unique_ptr<ContactParamModuleType>(nullptr);

            auto mpi = std::unique_ptr<MPIModuleType>(nullptr);

            return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis),std::move(mpi));
        };
    };
};

class RenderExecutionGraph;

namespace RenderLogicParserGenerators {

    struct LogicParserGen {

        LogicParserGen( RenderData * p, RenderExecutionGraph * g): m_p(p) , m_g(g){}

        RenderData * m_p;
        RenderExecutionGraph * m_g;

        template<typename TParser>
        typename TParser::ParserTraits::TupleModules
        createParserModules(TParser * p) {

            using ParserTraits = typename TParser::ParserTraits;
            using MaterialsModuleType = typename ParserTraits::template getModuleType<0>;
            using LogicModuleType     = typename ParserTraits::template getModuleType<1>;

            auto mat   = std::unique_ptr<MaterialsModuleType >(new MaterialsModuleType(p, &m_p->m_materials));
            auto logic = std::unique_ptr<LogicModuleType >    (new LogicModuleType(p, m_g, &m_p->m_geometryMap, &m_p->m_materials));

            return std::make_tuple(std::move(mat),std::move(logic));
        };

    };
};

#endif

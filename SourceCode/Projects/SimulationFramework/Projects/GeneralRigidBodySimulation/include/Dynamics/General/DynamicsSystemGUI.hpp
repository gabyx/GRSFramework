#ifndef DynamicsSystemGUI_hpp
#define DynamicsSystemGUI_hpp

#include <vector>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "DynamicsSystem.hpp"

#include <Ogre.h>


class DynamicsSystemGUI : public DynamicsSystemBase {
public:

    DynamicsSystemGUI( std::shared_ptr<Ogre::SceneManager> s ): m_pSceneMgr(s) {
        double lengthScale = 100;  // 1m = 100 Ogre units , 1cm -> 1 Ogre Unit
        m_pBaseNode = m_pSceneMgr->getSceneNode("BaseFrame")->createChildSceneNode("BaseFrameScene");
        m_pBaseNode->setScale(Ogre::Vector3(1.0,1.0,1.0)*lengthScale);
    }



    template<typename TParser>
    std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
        std::unique_ptr<typename TParser::ExternalForcesModuleType >,
        std::unique_ptr<typename TParser::ContactParamModuleType>,
        std::unique_ptr<typename TParser::InitStatesModuleType >,
        std::unique_ptr<typename TParser::BodyModuleType >,
        std::unique_ptr<typename TParser::GeometryModuleType >,
        std::unique_ptr<typename TParser::VisModuleType>
        >
    createParserModules(TParser * p) {

        using SettingsModuleType       = typename TParser::SettingsModuleType ;
        using ContactParamModuleType   = typename TParser::ContactParamModuleType;
        using GeometryModuleType       = typename TParser::GeometryModuleType ;
        using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
        using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
        using BodyModuleType           = typename TParser::BodyModuleType ;
        using VisModuleType            = typename TParser::VisModuleType ;


        auto sett = std::unique_ptr<SettingsModuleType >(new SettingsModuleType(p, &m_SettingsRecorder,
                    &m_SettingsTimestepper,
                    &m_SettingsInclusionSolver));

        auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &this->m_globalGeometries) );

        auto is  = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p,&this->m_bodiesInitStates, sett.get()));
        auto vis = std::unique_ptr<VisModuleType>( new VisModuleType(&m_SceneNodeSimBodies, &m_SceneNodeBodies, m_pBaseNode, m_pSceneMgr.get()) ); // no visualization needed
        auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  geom.get(), is.get(), vis.get() , &this->m_SimBodies, &this->m_Bodies )) ;
        auto es  = std::unique_ptr<ExternalForcesModuleType >(new ExternalForcesModuleType(p, &this->m_externalForces));
        auto con = std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p,&this->m_ContactParameterMap));

        return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis));
    }

public:

    struct RigidBodyGraphic {
        Ogre::SceneNode* m_node;
        RigidBodyIdType m_id;
    };

    using RigidBodyGraphicContType = std::vector<RigidBodyGraphic>;

    std::vector<RigidBodyGraphic>	m_SceneNodeSimBodies;
    std::vector<RigidBodyGraphic>	m_SceneNodeBodies;

private:
    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;

    Ogre::SceneNode * m_pBaseNode;
};



class DynamicsSystemPlayback {
public:
    DEFINE_DYNAMICSYSTEM_BASE_TYPES

    DynamicsSystemPlayback( std::shared_ptr<Ogre::SceneManager> s ): m_pSceneMgr(s) {
        double lengthScale = 100;  // 1m = 100 Ogre units , 1cm -> 1 Ogre Unit
        m_pBaseNode = m_pSceneMgr->getSceneNode("BaseFrame")->createChildSceneNode("BaseFrameScene");
        m_pBaseNode->setScale(Ogre::Vector3(1.0,1.0,1.0)*lengthScale);
    }


    template<typename TParser>
    std::tuple< std::unique_ptr<typename TParser::SettingsModuleType >,
        std::unique_ptr<typename TParser::ExternalForcesModuleType >,
        std::unique_ptr<typename TParser::ContactParamModuleType>,
        std::unique_ptr<typename TParser::InitStatesModuleType >,
        std::unique_ptr<typename TParser::BodyModuleType >,
        std::unique_ptr<typename TParser::GeometryModuleType >,
        std::unique_ptr<typename TParser::VisModuleType>
        >
    createParserModules(TParser * p) {

        using SettingsModuleType       = typename TParser::SettingsModuleType ;
        using ContactParamModuleType   = typename TParser::ContactParamModuleType;
        using GeometryModuleType       = typename TParser::GeometryModuleType ;
        using InitStatesModuleType     = typename TParser::InitStatesModuleType ;
        using ExternalForcesModuleType = typename TParser::ExternalForcesModuleType ;
        using BodyModuleType           = typename TParser::BodyModuleType ;
        using VisModuleType            = typename TParser::VisModuleType ;


        auto sett = std::unique_ptr<SettingsModuleType >(nullptr);

        auto geom = std::unique_ptr<GeometryModuleType >(new GeometryModuleType(p, &m_globalGeometries) );

        auto is  = std::unique_ptr<InitStatesModuleType >(new InitStatesModuleType(p,&m_bodiesInitStates, sett.get()));
        auto vis = std::unique_ptr<VisModuleType>( new VisModuleType(&m_SceneNodeSimBodies, &m_SceneNodeBodies, m_pBaseNode, m_pSceneMgr.get()) ); // no visualization needed
        auto bm  = std::unique_ptr<BodyModuleType>(new BodyModuleType(p,  geom.get(), is.get(), vis.get() , nullptr , nullptr )) ;
        auto es  = std::unique_ptr<ExternalForcesModuleType >(nullptr);
        auto con = std::unique_ptr<ContactParamModuleType>(nullptr);

        return std::make_tuple(std::move(sett),std::move(es),std::move(con),std::move(is),std::move(bm),std::move(geom),std::move(vis));
    }

    GlobalGeometryMapType m_globalGeometries;
    RigidBodyStatesContainerType m_bodiesInitStates;


    struct RigidBodyGraphic {
        Ogre::SceneNode* m_node;
        RigidBodyIdType m_id;
    };

    using RigidBodyGraphicContType = std::vector<RigidBodyGraphic>;

    std::vector<RigidBodyGraphic>	m_SceneNodeSimBodies;
    std::vector<RigidBodyGraphic>	m_SceneNodeBodies;

    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;

    Ogre::SceneNode * m_pBaseNode;
};


#endif

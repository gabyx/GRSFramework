// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_DynamicsSystemGUI_hpp
#define GRSF_dynamics_general_DynamicsSystemGUI_hpp

#include <vector>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include <boost/iterator/transform_iterator.hpp>

#include "GRSF/common/OgrePointCloud.hpp"

#include "GRSF/dynamics/general/DynamicsSystem.hpp"

#include "GRSF/systems/SceneParserGUI.hpp"

#include <Ogre.h>

class RigidBodyGraphics
{
public:
    RigidBodyGraphics() : m_node(nullptr), m_id(0)
    {
    }
    RigidBodyGraphics(Ogre::SceneNode* n, const RigidBodyIdType& id) : m_node(n), m_id(id)
    {
    }
    Ogre::SceneNode* m_node = nullptr;

    OgrePointCloud* m_pointCloud = nullptr;
    unsigned int    m_pointIdx   = 0;

    void setPointCloud(OgrePointCloud* pc, unsigned int idx)
    {
        m_pointCloud = pc;
        m_pointIdx   = idx;
    }

    RigidBodyIdType m_id;

    template <typename TRigidBodyState>
    inline void applyBodyState(const TRigidBodyState& s)
    {
        WARNINGMSG(m_id == s.m_id, "Updating non matching ids: " << m_id << "!=" << s.m_id);
        GRSF_ASSERTMSG(m_node, "SceneNode is null")
        if (m_pointCloud)
        {
            auto& p      = m_pointCloud->getPoint(m_pointIdx);
            p.position.x = s.m_q(0);
            p.position.y = s.m_q(1);
            p.position.z = s.m_q(2);
        }
        else
        {
            m_node->setPosition(s.m_q(0), s.m_q(1), s.m_q(2));
            m_node->setOrientation(s.m_q(6), s.m_q(3), s.m_q(4), s.m_q(5));
        }
    }
};

template <typename TRigidBodyGraphics>
class RigidBodyGraphicsContainer
{
private:
    using ContainerType = std::vector<TRigidBodyGraphics>;
    ContainerType m_mapByInsertion;

    template <typename Iter>
    struct KeyGetter : std::unary_function<const typename Iter::value_type, const RigidBodyIdType&>
    {
        inline const RigidBodyIdType& operator()(const typename Iter::value_type& pBody) const
        {
            return pBody.m_id;
        }
    };

public:
    RigidBodyGraphicsContainer()                                    = default;
    RigidBodyGraphicsContainer(const RigidBodyGraphicsContainer& o) = default;
    RigidBodyGraphicsContainer(RigidBodyGraphicsContainer&& o)      = default;
    RigidBodyGraphicsContainer& operator=(const RigidBodyGraphicsContainer& o) = default;
    RigidBodyGraphicsContainer& operator=(RigidBodyGraphicsContainer&& o) = default;

    using size_type = typename ContainerType::size_type;
    using iterator  = typename ContainerType::iterator;

    inline iterator begin()
    {
        return m_mapByInsertion.begin();
    }
    inline iterator end()
    {
        return m_mapByInsertion.end();
    }
    inline size_type size()
    {
        return m_mapByInsertion.size();
    }

    /** Key ierator ordered by insertion */
    using key_iterator = boost::transform_iterator<KeyGetter<iterator>, iterator>;

    /** Get key iterators ordered by insertion (random access) */
    key_iterator beginKey()
    {
        return key_iterator(m_mapByInsertion.begin(), KeyGetter<iterator>());
    }
    key_iterator endKey()
    {
        return key_iterator(m_mapByInsertion.end(), KeyGetter<iterator>());
    }

    template <class... Args>
    void addBody(Args&&... args)
    {
        m_mapByInsertion.emplace_back(std::forward<Args>(args)...);
    }

    template <typename TRigidBody>
    void addBody(TRigidBody&& b)
    {
        m_mapByInsertion.push_back(std::forward<TRigidBody>(b));
    }
};

class DynamicsSystemGUI : public DynamicsSystemBase
{
public:
    DynamicsSystemGUI(std::shared_ptr<Ogre::SceneManager> s, Ogre::SceneNode* pBaseNode)
        : m_pSceneMgr(s), m_pBaseNode(pBaseNode)
    {
        m_pBodiesNode        = pBaseNode->createChildSceneNode("BaseFrameBodies");
        m_pContactFramesNode = pBaseNode->createChildSceneNode("BaseContactFrames");
    }

    ~DynamicsSystemGUI()
    {
    }

    inline void afterFirstTimeStep(){};
    inline void afterSecondTimeStep(){};
    void doInputTimeStep(PREC T){};

    struct ParserModulesCreator
    {
        ParserModulesCreator(DynamicsSystemGUI* p) : m_p(p)
        {
        }
        DynamicsSystemGUI* m_p;

        template <typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser, TDynamicsSystem>
        {
            // Module typedefs
            using SettingsModuleType       = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType   = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType     = ParserModules::InitStatesModule<SceneParserTraits>;

            using BodyMStaticOptions = ParserModules::BodyModuleStaticOptions<>;
            using BodyModuleType     = ParserModules::BodyModule<SceneParserTraits, BodyMStaticOptions>;

            using GeomMStaticOptions = ParserModules::GeometryModuleStaticOptions<true, true, true, false>;
            using GeometryModuleType = ParserModules::GeometryModule<SceneParserTraits, GeomMStaticOptions>;

            using VisModuleType = ParserModules::VisModule<SceneParserTraits>;
            using MPIModuleType = ParserModules::MPIModuleDummy<SceneParserTraits>;
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

            auto vis = std::unique_ptr<VisModuleType>(new VisModuleType(p,
                                                                        &m_p->m_SceneNodeSimBodies,
                                                                        &m_p->m_SceneNodeBodies,
                                                                        m_p->m_pBaseNode,
                                                                        m_p->m_pBodiesNode,
                                                                        m_p->m_pContactFramesNode,
                                                                        m_p->m_pSceneMgr.get()));
            auto geom = std::unique_ptr<GeometryModuleType>(new GeometryModuleType(
                p, &m_p->m_globalGeometries, vis->getScalesGroup()));  // geom module needs to track

            auto is = std::unique_ptr<InitStatesModuleType>(
                new InitStatesModuleType(p, &m_p->m_bodiesInitStates, &m_p->m_settingsTimestepper));
            auto bm = std::unique_ptr<BodyModuleType>(
                new BodyModuleType(p, geom.get(), is.get(), vis.get(), &m_p->m_simBodies, &m_p->m_staticBodies));
            auto es =
                std::unique_ptr<ExternalForcesModuleType>(new ExternalForcesModuleType(p, &m_p->m_externalForces));
            auto con =
                std::unique_ptr<ContactParamModuleType>(new ContactParamModuleType(p, &m_p->m_ContactParameterMap));

            auto mpi = std::unique_ptr<MPIModuleType>(nullptr);

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

public:
    using RigidBodyGraphicsType = RigidBodyGraphics;

    using RigidBodyGraphicsContType = RigidBodyGraphicsContainer<RigidBodyGraphicsType>;

    RigidBodyGraphicsContType m_SceneNodeSimBodies;
    RigidBodyGraphicsContType m_SceneNodeBodies;

    Ogre::SceneNode* m_pContactFramesNode;

private:
    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;

    Ogre::SceneNode* m_pBodiesNode;
    Ogre::SceneNode* m_pBaseNode;
};

class DynamicsSystemPlayback
{
public:
    DEFINE_DYNAMICSYSTEM_BASE_TYPES

    DynamicsSystemPlayback(std::shared_ptr<Ogre::SceneManager> s, Ogre::SceneNode* pBaseNode)
        : m_pSceneMgr(s), m_pBaseNode(pBaseNode)
    {
        m_pBodiesNode        = pBaseNode->createChildSceneNode("BaseFrameBodies");
        m_pContactFramesNode = pBaseNode->createChildSceneNode("BaseContactFrames");
    }

    ~DynamicsSystemPlayback()
    {
    }

    struct ParserModulesCreator
    {
        ParserModulesCreator(DynamicsSystemPlayback* p) : m_p(p)
        {
        }
        DynamicsSystemPlayback* m_p;

        template <typename TSceneParser, typename TDynamicsSystem>
        struct SceneParserTraits : SceneParserBaseTraits<TSceneParser, TDynamicsSystem>
        {
            // Module typedefs
            using SettingsModuleType       = ParserModules::SettingsModule<SceneParserTraits>;
            using ExternalForcesModuleType = ParserModules::ExternalForcesModule<SceneParserTraits>;
            using ContactParamModuleType   = ParserModules::ContactParamModule<SceneParserTraits>;
            using InitStatesModuleType     = ParserModules::InitStatesModule<SceneParserTraits>;

            using BodyMStaticOptions =
                ParserModules::BodyModuleStaticOptions<true, true, true, true, false, false, true>;
            using BodyModuleType = ParserModules::BodyModule<SceneParserTraits, BodyMStaticOptions>;

            using GeomMStaticOptions = ParserModules::GeometryModuleStaticOptions<true, true, true, false>;
            using GeometryModuleType = ParserModules::GeometryModule<SceneParserTraits, GeomMStaticOptions>;

            using VisModuleType = ParserModules::VisModule<SceneParserTraits>;
            using MPIModuleType = ParserModules::MPIModuleDummy<SceneParserTraits>;
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

            auto sett = std::unique_ptr<SettingsModuleType>(nullptr);

            auto vis =
                std::unique_ptr<VisModuleType>(new VisModuleType(p,
                                                                 &m_p->m_SceneNodeSimBodies,
                                                                 &m_p->m_SceneNodeBodies,
                                                                 m_p->m_pBaseNode,
                                                                 m_p->m_pBodiesNode,
                                                                 m_p->m_pContactFramesNode,
                                                                 m_p->m_pSceneMgr.get()));  // no visualization needed
            auto geom = std::unique_ptr<GeometryModuleType>(
                new GeometryModuleType(p, &m_p->m_globalGeometries, vis->getScalesGroup()));

            auto is =
                std::unique_ptr<InitStatesModuleType>(new InitStatesModuleType(p, &m_p->m_bodiesInitStates, nullptr));
            auto bm = std::unique_ptr<BodyModuleType>(
                new BodyModuleType(p, geom.get(), is.get(), vis.get(), nullptr, nullptr));
            auto es  = std::unique_ptr<ExternalForcesModuleType>(nullptr);
            auto con = std::unique_ptr<ContactParamModuleType>(nullptr);

            auto mpi = std::unique_ptr<MPIModuleType>(nullptr);

            return std::make_tuple(std::move(sett),
                                   std::move(es),
                                   std::move(con),
                                   std::move(is),
                                   std::move(bm),
                                   std::move(geom),
                                   std::move(vis),
                                   std::move(mpi));
        };
    };

    GlobalGeometryMapType        m_globalGeometries;
    RigidBodyStatesContainerType m_bodiesInitStates;

    using RigidBodyGraphicsType     = RigidBodyGraphics;
    using RigidBodyGraphicsContType = RigidBodyGraphicsContainer<RigidBodyGraphicsType>;

    RigidBodyGraphicsContType m_SceneNodeSimBodies;
    RigidBodyGraphicsContType m_SceneNodeBodies;

    Ogre::SceneNode* m_pContactFramesNode;

private:
    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;

    Ogre::SceneNode* m_pBodiesNode;
    Ogre::SceneNode* m_pBaseNode;
};

#endif

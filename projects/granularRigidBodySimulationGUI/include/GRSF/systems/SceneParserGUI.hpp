// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_systems_SceneParserGUI_hpp
#define GRSF_systems_SceneParserGUI_hpp

#include <OGRE/Ogre.h>
#include "GRSF/common/AxisObject.hpp"
#include "GRSF/common/OgrePointCloud.hpp"

#include "GRSF/systems/SceneParser.hpp"

#include "GRSF/dynamics/general/MakeCoordinateSystem.hpp"

namespace ParserModules
{
template <typename TParserTraits>
class VisModule
{
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits)

    DEFINE_MATRIX_TYPES

    using StatesGroupType = typename InitStatesModuleType::StatesGroupType;
    StatesGroupType* m_statesGroup;

    using BodyMode = typename DynamicsSystemType::RigidBodyType::BodyMode;
    BodyMode m_mode;
    using BodyListType            = typename BodyModuleType::BodyListType;
    BodyListType* m_bodyListGroup = nullptr;
    RigidBodyIdType m_startIdGroup;

    using RigidBodyGraphicsType     = typename DynamicsSystemType::RigidBodyGraphicsType;
    using RigidBodyGraphicsContType = typename DynamicsSystemType::RigidBodyGraphicsContType;
    RigidBodyGraphicsContType* m_pSimBodies;
    RigidBodyGraphicsContType* m_pBodies;

    Ogre::SceneManager* m_pSceneMgr;
    Ogre::SceneNode* m_pBaseNode;
    Ogre::SceneNode* m_pBodiesNode;
    Ogre::SceneNode* m_pContactFramesNode;

    LogType* m_pSimulationLog;

    struct RenderSettings
    {
        bool attachAxis     = false;
        double axesSize     = 1;
        bool shadowsEnabled = true;
    };

    std::vector<std::string> m_materialList;

    using ScalesList = typename GeometryModuleType::ScalesList;
    ScalesList m_scalesGroup;

    Vector3 m_currScale;
    bool m_currScaleLikeGeom;

public:
    ScalesList* getScalesGroup()
    {
        return &m_scalesGroup;
    }

    void cleanUp()
    {
        m_materialList.clear();
        m_scalesGroup.clear();
    }

    VisModule(ParserType* p,
              RigidBodyGraphicsContType* pSimBodies,
              RigidBodyGraphicsContType* pBodies,
              Ogre::SceneNode* pBaseNode,
              Ogre::SceneNode* pBodiesNode,
              Ogre::SceneNode* pContactFramesNode,
              Ogre::SceneManager* pSceneMgr)
        : m_pSimulationLog(p->getSimLog())
        , m_pSimBodies(pSimBodies)
        , m_pBodies(pBodies)
        , m_pBaseNode(pBaseNode)
        , m_pBodiesNode(pBodiesNode)
        , m_pContactFramesNode(pContactFramesNode)
        , m_pSceneMgr(pSceneMgr){GRSF_ASSERTMSG(m_pSceneMgr && m_pBodiesNode, "these should not be zero!")

          };

    void parse(XMLNodeType vis, BodyListType* bodyList, StatesGroupType* states, RigidBodyIdType startId, BodyMode mode)
    {
        m_mode          = mode;
        m_startIdGroup  = startId;
        m_bodyListGroup = bodyList;
        m_statesGroup   = states;

        GRSF_ASSERTMSG(bodyList->size() == m_scalesGroup.size(),
                       "The scales list has not been filled, this needs to be done outside of this module!")

        LOGSCLEVEL1(m_pSimulationLog, "---> VisModule: parsing (BodyVisualization)" << std::endl;)
        XMLNodeType node = vis.first_child();
        std::string n    = node.name();

        auto p              = VisSubModuleScale::parseScale(node, n);
        m_currScaleLikeGeom = p.first;
        m_currScale         = p.second;

        if (n == "Plane")
        {
            parsePlane(node);
        }
        else if (n == "PointCloud")
        {
            parsePointCloud(node);
        }
        else if (n == "Mesh")
        {
            parseMesh(node);
        }
        else if (n == "Capsule")
        {
            parseCapsule(node);
        }
        else
        {
            GRSF_ERRORMSG("Visualization for type: " << n << " not implemented!")
        }
    }

    void parseSceneSettingsPost(XMLNodeType sceneSettings)
    {
        LOGSCLEVEL1(m_pSimulationLog,
                    "==== VisModule: parsing (SceneSettingsPost, MPI/Visualization)  ===" << std::endl;)

        parseMPISettings(sceneSettings);
        parseSceneVisualizationSettings(sceneSettings);

        LOGSCLEVEL1(m_pSimulationLog,
                    "===================================================================" << std::endl;)
    }

private:
    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void parseMesh(XMLNodeType meshNode)
    {
        XMLAttributeType att;
        //        static int nodeCounter = 0;
        //        static int entityCounter = 0;

        GET_XMLATTRIBUTE_CHECK(att, "file", meshNode);
        boost::filesystem::path meshName = att.value();

        XMLNodeType rendering = meshNode.child("Rendering");
        RenderSettings renderSettings;
        if (rendering)
        {
            parseRenderSettings(rendering, renderSettings);
        }
        LOGSCLEVEL1(
            m_pSimulationLog,
            "---> RenderSettings: axis= " << renderSettings.attachAxis << " shadows: " << renderSettings.shadowsEnabled
                                          << std::endl;)

        parseMaterials(meshNode);

        std::stringstream entity_name, node_name;
        LOG(m_pSimulationLog, "---> Add all Ogre Mesh Objects" << std::endl);

        unsigned int i;  // Linear offset form the m_startIdGroup

        for (unsigned int bodyIdx = 0; bodyIdx < m_bodyListGroup->size(); ++bodyIdx)
        {
            auto id = (*m_bodyListGroup)[bodyIdx].m_id;
            i       = id - m_startIdGroup;

            Ogre::Entity* ent = m_pSceneMgr->createEntity(meshName.string());
            ent->setCastShadows(renderSettings.shadowsEnabled);
            Ogre::SceneNode* sceneNode      = m_pBodiesNode->createChildSceneNode();
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();

            RigidBodyGraphicsType rigidBodyGraphics(sceneNode, id);

            if (m_currScaleLikeGeom)
            {
                auto& s = m_scalesGroup[bodyIdx];
                if (s(0) == 0 || s(1) == 0 || s(2) == 0)
                {
                    GRSF_ERRORMSG("---> parseMesh:: Scale for Mesh: " + meshName.string() + "is zero or smaller!");
                }
                sceneNodeScale->setScale(s(0), s(1), s(2));
            }
            else
            {
                if (m_currScale(0) == 0 || m_currScale(1) == 0 || m_currScale(2) == 0)
                {
                    GRSF_ERRORMSG("---> parseMesh:: Scale for Mesh: " + meshName.string() + "is zero or smaller!");
                }
                sceneNodeScale->setScale(m_currScale(0), m_currScale(1), m_currScale(2));
            }
            sceneNodeScale->attachObject(ent);

            if (renderSettings.attachAxis)
            {
                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode();
                Ogre::Entity* axisEnt          = m_pSceneMgr->createEntity("axes.mesh");
                // Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() +
                // "Axes",100);
                sceneNodeAxes->setScale(renderSettings.axesSize, renderSettings.axesSize, renderSettings.axesSize);
                sceneNodeAxes->attachObject(axisEnt);
            }

            unsigned int matIdx = i % m_materialList.size();

            ent->setMaterialName(m_materialList[matIdx]);

            // Set initial condition
            rigidBodyGraphics.applyBodyState((*m_statesGroup)[bodyIdx]);

            if (m_mode == BodyMode::SIMULATED)
            {
                m_pSimBodies->addBody(rigidBodyGraphics);
            }
            else if (m_mode == BodyMode::STATIC)
            {
                m_pBodies->addBody(rigidBodyGraphics);
            }

            //            ++nodeCounter;
            //            ++entityCounter;
        }
    }

    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void parseCapsule(XMLNodeType capsule)
    {
        XMLAttributeType att;

        GET_XMLATTRIBUTE_CHECK(att, "fileZylinder", capsule);
        boost::filesystem::path fileZyl = att.value();

        GET_XMLATTRIBUTE_CHECK(att, "fileCap", capsule);
        boost::filesystem::path fileCap = att.value();

        XMLNodeType rendering = capsule.child("Rendering");
        RenderSettings renderSettings;
        if (rendering)
        {
            parseRenderSettings(rendering, renderSettings);
        }
        LOGSCLEVEL1(
            m_pSimulationLog,
            "---> RenderSettings: axis= " << renderSettings.attachAxis << " shadows: " << renderSettings.shadowsEnabled
                                          << std::endl;)

        parseMaterials(capsule);

        std::stringstream entity_name, node_name;
        LOG(m_pSimulationLog, "---> Add Ogre Capsule Objects" << std::endl);

        unsigned int i;  // Linear offset form the m_startIdGroup

        for (unsigned int bodyIdx = 0; bodyIdx < m_bodyListGroup->size(); ++bodyIdx)
        {
            auto id = (*m_bodyListGroup)[bodyIdx].m_id;
            i       = id - m_startIdGroup;

            Ogre::Entity* entZyl  = m_pSceneMgr->createEntity(/*entity_name.str(),*/ fileZyl.string());
            Ogre::Entity* entCap1 = m_pSceneMgr->createEntity(/*entity_name.str(),*/ fileCap.string());
            Ogre::Entity* entCap2 = m_pSceneMgr->createEntity(/*entity_name.str(),*/ fileCap.string());
            entZyl->setCastShadows(renderSettings.shadowsEnabled);
            entCap1->setCastShadows(renderSettings.shadowsEnabled);
            entCap2->setCastShadows(renderSettings.shadowsEnabled);

            Ogre::SceneNode* sceneNode = m_pBodiesNode->createChildSceneNode(/*node_name.str()*/);
            RigidBodyGraphicsType rigidBodyGraphics(sceneNode, id);

            Ogre::SceneNode* sceneNodeZyl = sceneNode->createChildSceneNode();
            sceneNodeZyl->attachObject(entZyl);
            Ogre::SceneNode* sceneNodeCap1 = sceneNode->createChildSceneNode();
            sceneNodeCap1->attachObject(entCap1);
            Ogre::SceneNode* sceneNodeCap2 = sceneNode->createChildSceneNode();
            sceneNodeCap2->attachObject(entCap2);

            if (m_currScaleLikeGeom)
            {
                auto& s = m_scalesGroup[bodyIdx];
                if (s(0) <= 0 || s(1) <= 0 || s(2) <= 0)
                {
                    GRSF_ERRORMSG("---> parseCapsule:: Scale for Mesh: " << fileZyl.string() << ", " << fileCap.string()
                                                                         << " is zero or smaller!");
                }
                sceneNodeZyl->setScale(s(0), s(1), s(2));
                sceneNodeCap1->setScale(s(0), s(1), s(2));
                sceneNodeCap2->setScale(s(0), s(1), s(2));
            }
            else
            {
                sceneNodeZyl->setScale(m_currScale(0), m_currScale(1), m_currScale(2));
                sceneNodeCap1->setScale(m_currScale(0), m_currScale(1), std::min(m_currScale(0), m_currScale(1)));
                sceneNodeCap2->setScale(m_currScale(0), m_currScale(1), std::min(m_currScale(0), m_currScale(1)));

                sceneNodeCap1->setPosition(0, 0, 0.5 * m_currScale(2));
                sceneNodeCap2->setPosition(0, 0, -0.5 * m_currScale(2));
                sceneNodeCap2->setOrientation(Ogre::Quaternion(Ogre::Radian(M_PI), Ogre::Vector3(1, 0, 0)));
            }

            if (renderSettings.attachAxis)
            {
                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode();
                Ogre::Entity* axisEnt          = m_pSceneMgr->createEntity("axes.mesh");
                // Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() +
                // "Axes",100);
                sceneNodeAxes->setScale(renderSettings.axesSize, renderSettings.axesSize, renderSettings.axesSize);
                sceneNodeAxes->attachObject(axisEnt);
            }

            unsigned int matIdx = i % m_materialList.size();

            entZyl->setMaterialName(m_materialList[matIdx]);
            entCap1->setMaterialName(m_materialList[matIdx]);
            entCap2->setMaterialName(m_materialList[matIdx]);

            // Set initial condition
            rigidBodyGraphics.applyBodyState((*m_statesGroup)[bodyIdx]);

            if (m_mode == BodyMode::SIMULATED)
            {
                m_pSimBodies->addBody(rigidBodyGraphics);
            }
            else if (m_mode == BodyMode::STATIC)
            {
                m_pBodies->addBody(rigidBodyGraphics);
            }
        }
    }

    void parsePointCloud(XMLNodeType pcloudNode)
    {
        static int nodeCounter = 0;
        std::stringstream node_name;

        XMLAttributeType att;

        Vector4 color;
        att = pcloudNode.attribute("color");
        if (att)
        {
            if (!Utilities::stringToType(color, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion in parsePointCloud: scale failed");
            }
        }

        enum RenderMode
        {
            RM_POINTS,
            RM_SQUARES,
            RM_FLAT_SQUARES,
            RM_SPHERES,
            RM_TILES,
            RM_BOXES,
        };
        std::string rm;
        OgrePointCloud::RenderMode renderMode;
        att = pcloudNode.attribute("renderMode");
        if (att)
        {
            rm = att.value();
            if (rm == "points")
            {
                renderMode = OgrePointCloud::RM_POINTS;
            }
            else if (rm == "spheres")
            {
                renderMode = OgrePointCloud::RM_SPHERES;
            }
            else if (rm == "flatSquares")
            {
                renderMode = OgrePointCloud::RM_FLAT_SQUARES;
            }
            else if (rm == "squares")
            {
                renderMode = OgrePointCloud::RM_SQUARES;
            }
            else if (rm == "tiles")
            {
                renderMode = OgrePointCloud::RM_TILES;
            }
            else if (rm == "boxes")
            {
                renderMode = OgrePointCloud::RM_BOXES;
            }
        }

        node_name.str("");
        node_name << "PointCloud" << nodeCounter++;

        // Add Scene node for points
        Ogre::SceneNode* sceneNode = m_pBodiesNode->createChildSceneNode(node_name.str());
        // Add point cloud
        OgrePointCloud* pc = new OgrePointCloud();
        pc->setDimensions(m_currScale(0), m_currScale(1), m_currScale(2));
        pc->setRenderMode(renderMode);
        sceneNode->attachObject(pc);

        std::vector<OgrePointCloud::Point> points(m_bodyListGroup->size());

        auto stateIt = m_statesGroup->begin();
        // unsigned int i; // linear offset from m_startIdGroup

        unsigned int bodyCounter = 0;
        for (auto& b : *m_bodyListGroup)
        {
            // i = b.m_id - m_startIdGroup;  // not yet needed

            RigidBodyGraphicsType rigidBodyGraphics(sceneNode, b.m_id);
            rigidBodyGraphics.setPointCloud(pc, bodyCounter);  // each

            points[bodyCounter].position.x = stateIt->m_q(0);
            points[bodyCounter].position.y = stateIt->m_q(1);
            points[bodyCounter].position.z = stateIt->m_q(2);
            points[bodyCounter].color      = Ogre::ColourValue(color(0), color(1), color(2), color(3));

            if (m_mode == BodyMode::SIMULATED)
            {
                m_pSimBodies->addBody(rigidBodyGraphics);
            }
            else if (m_mode == BodyMode::STATIC)
            {
                m_pBodies->addBody(rigidBodyGraphics);
            }
            ++stateIt;
            ++bodyCounter;
        }

        // Add empty points to point cloud
        pc->addPoints(points.data(), points.size());
    }

    void parsePlane(XMLNodeType planeNode)
    {
        XMLAttributeType att;
        static int nodeCounter   = 0;
        static int entityCounter = 0;

        Vector2 subDivs = Vector2::Ones();
        att             = planeNode.attribute("subDivisions");
        if (att)
        {
            if (!Utilities::stringToType(subDivs, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion in parsePlane: subDivisions failed");
            }
        }

        Vector3 normal;
        normal(0) = 0;
        normal(1) = 0;
        normal(2) = 1;
        att       = planeNode.attribute("normal");
        if (att)
        {
            if (!Utilities::stringToType(normal, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion in parsePlane: normal failed");
            }
        }

        PREC dist = 0;
        att       = planeNode.attribute("distance");
        if (att)
        {
            if (!Utilities::stringToType(dist, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion in parsePlane: distance failed");
            }
        }

        Vector2 tile;
        tile(0) = 1;
        tile(1) = 1;
        att     = planeNode.attribute("tileTexture");
        if (att)
        {
            if (!Utilities::stringToType(tile, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion in parsePlane: tileTexture failed");
            }
        }

        XMLNodeType rendering = planeNode.child("Rendering");
        RenderSettings renderSettings;
        if (rendering)
        {
            parseRenderSettings(rendering, renderSettings);
        }
        LOGSCLEVEL1(
            m_pSimulationLog,
            "---> RenderSettings: axis= " << renderSettings.attachAxis << " shadows: " << renderSettings.shadowsEnabled
                                          << std::endl;)

        parseMaterials(planeNode);

        std::stringstream entity_name, node_name, plane_name;
        LOGSCLEVEL1(m_pSimulationLog, "---> Add all Ogre: Plane Objects" << std::endl);

        unsigned int i;  // linear offset from m_startIdGroup
        auto stateIt = m_statesGroup->begin();
        for (auto& b : *m_bodyListGroup)
        {
            i = b.m_id - m_startIdGroup;

            entity_name.str("");
            node_name.str("");
            entity_name << "OgrePlane" << std::string("Entity");
            node_name << "OgrePlane" << std::string("Node");
            plane_name << "OgrePlane";
            entity_name << entityCounter;
            node_name << nodeCounter;
            plane_name << nodeCounter;
            // TODO m_pSceneMgr->createInstancedGeometry();

            // Make mesh

            Ogre::Plane plane;
            plane.normal = Ogre::Vector3(normal(0), normal(1), normal(2));
            plane.d      = dist;

            // Do some calculation becaus eOgre nees a correct UpVector ...
            Vector3 v1, v2;
            CoordinateSystem::makeCoordinateSystem(normal, v1, v2);

            Ogre::MeshManager::getSingleton().createPlane(plane_name.str(),
                                                          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          plane,
                                                          m_currScale(0),
                                                          m_currScale(1),
                                                          subDivs(0),
                                                          subDivs(1),
                                                          true,
                                                          1,
                                                          tile(0),
                                                          tile(1),
                                                          Ogre::Vector3(v1(0), v1(1), v1(2)));

            Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(), plane_name.str());
            ent->setCastShadows(renderSettings.shadowsEnabled);

            Ogre::SceneNode* sceneNode      = m_pBodiesNode->createChildSceneNode(node_name.str());
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();

            RigidBodyGraphicsType rigidBodyGraphics(sceneNode, b.m_id);

            if (m_currScaleLikeGeom)
            {
                GRSF_ERRORMSG("---> parsePlane:: Scale for Plane can not be used from Geometry!");
            }
            else
            {
                sceneNodeScale->setScale(m_currScale(0), m_currScale(1), m_currScale(2));
            }
            sceneNodeScale->attachObject(ent);

            if (renderSettings.attachAxis)
            {
                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
                Ogre::Entity* axisEnt          = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt", "axes.mesh");
                // Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() +
                // "Axes",100);
                sceneNodeAxes->setScale(renderSettings.axesSize, renderSettings.axesSize, renderSettings.axesSize);
                sceneNodeAxes->attachObject(axisEnt);
            }

            int matIdx = i % m_materialList.size();

            ent->setMaterialName(m_materialList[matIdx]);

            // Set initial condition
            rigidBodyGraphics.applyBodyState(*stateIt);

            if (m_mode == BodyMode::SIMULATED)
            {
                m_pSimBodies->addBody(rigidBodyGraphics);
            }
            else if (m_mode == BodyMode::STATIC)
            {
                m_pBodies->addBody(rigidBodyGraphics);
            }

            ++nodeCounter;
            ++entityCounter;
            ++stateIt;
        }
    }

    void parseMaterials(XMLNodeType meshNode)
    {
        std::string type = meshNode.attribute("type").value();
        if (type == "permutate" || type == "uniform")
        {
            m_materialList.clear();
            // Iterate over all material, save in list!
            for (auto& n : meshNode.children("Material"))
            {
                m_materialList.emplace_back(n.attribute("name").value());
            }
            if (m_materialList.empty())
            {
                GRSF_ERRORMSG("---> No Material Node found in Mesh!");
            }
        }
        else
        {
            GRSF_ERRORMSG("---> The attribute 'type' '" << type
                                                        << "' of 'parseMesh' has no implementation in the parser");
        }
    }

    void parseRenderSettings(XMLNodeType rendering, RenderSettings& settings)
    {
        XMLAttributeType att;
        att = rendering.attribute("attachAxis");
        if (att)
        {
            if (!Utilities::stringToType(settings.attachAxis, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion of in parseMesh: attachAxis failed");
            }
        }
        att = rendering.attribute("axesSize");
        if (att)
        {
            if (!Utilities::stringToType(settings.axesSize, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion of in parseMesh: axesSize failed");
            }
        }

        att = rendering.attribute("shadowsEnabled");
        if (att)
        {
            if (!Utilities::stringToType(settings.shadowsEnabled, att.value()))
            {
                GRSF_ERRORMSG("---> String conversion of in parseMesh: shadowsEnabled failed");
            }
        }
    }

    void parseMPISettings(XMLNodeType sceneSettings)
    {
        XMLNodeType mpiSettings = sceneSettings.child("MPISettings");
        if (mpiSettings)
        {
            XMLNodeType topo = mpiSettings.child("ProcessTopology").child("Topology");
            CHECK_XMLNODE(topo, "ProcessTopology/Topology");

            std::string type = topo.attribute("type").value();
            if (type == "grid")
            {
                Vector3 minPoint, maxPoint;
                if (!Utilities::stringToType(minPoint, topo.attribute("minPoint").value()))
                {
                    GRSF_ERRORMSG("---> String conversion in parseMPISettings: minPoint failed");
                }
                if (!Utilities::stringToType(maxPoint, topo.attribute("maxPoint").value()))
                {
                    GRSF_ERRORMSG("---> String conversion in parseMPISettings: maxPoint failed");
                }

                MyMatrix::Array3<unsigned int> dim;
                if (!Utilities::stringToType(dim, topo.attribute("dimension").value()))
                {
                    GRSF_ERRORMSG("---> String conversion in parseMPISettings: dimension failed");
                }

                Vector3 extent;
                extent.array() = (maxPoint - minPoint);
                Vector3 center = 0.5 * extent + minPoint;
                Vector3 dxyz;
                dxyz.array() = extent.array() / dim.array().cast<PREC>();

                // Add MPI Visulaization
                std::string materialName = mpiSettings.child("ProcessTopology")
                                               .child("Visualization")
                                               .child("Material")
                                               .attribute("name")
                                               .value();
                Ogre::SceneNode* mpiTopoRoot = m_pBaseNode->createChildSceneNode("MPIProcTopo");

                // Add subdivision in each direction
                Matrix33 I = Matrix33::Zero();
                I(1, 0) = 1;
                I(0, 1) = 1;
                I(2, 2) = 1;

                std::stringstream name;
                name.str("");
                name << "MPITopoSubGridManual";
                Ogre::ManualObject* manual = m_pSceneMgr->createManualObject(name.str());

                manual->begin(materialName, Ogre::RenderOperation::OT_LINE_LIST);
                for (unsigned int k = 0; k < dim(1) + 1; k++)
                {
                    for (unsigned int l = 0; l < dim(2) + 1; l++)
                    {
                        manual->position(minPoint(0), minPoint(1) + k * dxyz(1), minPoint(2) + l * dxyz(2));
                        manual->position(maxPoint(0), minPoint(1) + k * dxyz(1), minPoint(2) + l * dxyz(2));
                    }
                }

                for (unsigned int k = 0; k < dim(0) + 1; k++)
                {
                    for (unsigned int l = 0; l < dim(1) + 1; l++)
                    {
                        manual->position(minPoint(0) + k * dxyz(0), minPoint(1) + l * dxyz(1), minPoint(2));
                        manual->position(minPoint(0) + k * dxyz(0), minPoint(1) + l * dxyz(1), maxPoint(2));
                    }
                }
                for (unsigned int k = 0; k < dim(2) + 1; k++)
                {
                    for (unsigned int l = 0; l < dim(0) + 1; l++)
                    {
                        manual->position(minPoint(0) + l * dxyz(0), minPoint(1), minPoint(2) + k * dxyz(2));
                        manual->position(minPoint(0) + l * dxyz(0), maxPoint(1), minPoint(2) + k * dxyz(2));
                    }
                }
                manual->end();

                mpiTopoRoot->attachObject(manual);

                //                    Ogre::SceneNode* mpiTopoRoot = m_BaseFrame->createChildSceneNode("MPIProcTopo");
                //                    Ogre::SceneNode* mpiTopoBase =
                //                    mpiTopoRoot->createChildSceneNode("MPIProcTopoBase");
                //                    Ogre::Entity *ent = m_pSceneMgr->createEntity("MPITopoBaseGrid", "Cube.mesh");
                //                    ent->setMaterialName(materialName);
                //                    mpiTopoBase->attachObject(ent);
                //                    mpiTopoBase->setPosition(center(0),center(1),center(2));
                //                    mpiTopoBase->setScale(0.5*extent(0),0.5*extent(1),0.5*extent(2));
                //
                //                    //Add subdivision in each direction
                //                    Matrix33 I = Matrix33::Zero();
                //                    I(1,0)=1; I(0,1)=1; I(2,2)=1;
                //
                //                    std::stringstream ent_name;
                //                    for(int i=0;i<3;i++){
                //                        Vector3 centerDivs = center;
                //                        centerDivs(i) -=  0.5*extent(i);
                //                        for(int k=1;k<dim(i);k++){
                //                            ent_name.str(""); ent_name << "MPITopoSubGridNode-" << i<<"-"<<k;
                //                            Ogre::SceneNode* mpiTopoSubdivs =
                //                            mpiTopoRoot->createChildSceneNode(ent_name.str());
                //                            ent_name.str(""); ent_name << "MPITopoSubGridEnt-" << i<<"-"<<k;
                //                            Ogre::Entity *ent = m_pSceneMgr->createEntity(ent_name.str(),
                //                            "Plane.mesh");
                //                            ent->setMaterialName(materialName);
                //                            mpiTopoSubdivs->attachObject(ent);
                //
                //
                //                            mpiTopoSubdivs->rotate(Ogre::Vector3(I(0,i),I(1,i),I(2,i)),Ogre::Degree(90));
                //                            if(i==0){
                //                                mpiTopoSubdivs->scale(0.5*extent(2),0.5*extent(1),1);
                //                            }else if(i==1){
                //                                mpiTopoSubdivs->scale(0.5*extent(1),0.5*extent(2),1);
                //                            }else{
                //                                mpiTopoSubdivs->scale(0.5*extent(0),0.5*extent(1),1);
                //                            }
                //                            centerDivs(i) += dxyz(i);
                //                            mpiTopoSubdivs->setPosition(centerDivs(0),centerDivs(1),centerDivs(2));
                //
                //                        }
                //
                //                    }
            }
            else
            {
                WARNINGMSG(false, "---> MPISettings:parseTopology: type: " << type << "not supported!");
            }
        }
    }
    void parseSceneVisualizationSettings(XMLNodeType sceneSettings)
    {
        XMLNodeType sceneVisSettings = sceneSettings.child("Visualization");
        if (sceneVisSettings)
        {
            XMLNodeType n = sceneVisSettings.child("Scene");
            if (n)
            {
                double scale;
                if (!Utilities::stringToType(scale, n.attribute("scale").value()))
                {
                    GRSF_ERRORMSG("---> String conversion in ContactCoordinateSystem: scale failed");
                }
                m_pBaseNode->setScale(Ogre::Vector3(1, 1, 1) * scale);
            }

            n = sceneVisSettings.child("ContactCoordinateSystem");
            if (n)
            {
                double scale;
                if (!Utilities::stringToType(scale, n.attribute("scale").value()))
                {
                    GRSF_ERRORMSG("---> String conversion in ContactCoordinateSystem: scale failed");
                }
                m_pContactFramesNode->setScale(Ogre::Vector3(1, 1, 1) * scale);
            }
        }
    }
};
};

template <typename TDynamicsSystem, template <typename P, typename D> class TParserTraits>
class SceneParserGUI
    : public SceneParser<TDynamicsSystem, TParserTraits, SceneParserGUI<TDynamicsSystem, TParserTraits>>
{
private:
    using BaseType = SceneParser<TDynamicsSystem, TParserTraits, SceneParserGUI<TDynamicsSystem, TParserTraits>>;

public:
    using DynamicsSystemType = TDynamicsSystem;

public:
    template <typename ModuleGeneratorType>
    SceneParserGUI(ModuleGeneratorType& moduleGen, Logging::Log* log, const boost::filesystem::path& mediaDir)
        : BaseType(moduleGen, log, mediaDir)
    {
    }
};

#endif

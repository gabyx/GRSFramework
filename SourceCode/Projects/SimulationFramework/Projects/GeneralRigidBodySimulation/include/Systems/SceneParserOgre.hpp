#ifndef SceneParserOgre_hpp
#define SceneParserOgre_hpp

#include <OGRE/Ogre.h>

#include "SceneParser.hpp"

template<typename TConfig>
class SceneParserOgre : public SceneParser<TConfig> {
public:

    DEFINE_CONFIG_TYPES_OF(TConfig)

    SceneParserOgre(
        Ogre::SceneNode * baseFrame,
        boost::shared_ptr<Ogre::SceneManager> pSceneMgr,
        std::vector<Ogre::SceneNode*> &nodesSimBodies,
        std::vector<Ogre::SceneNode*> &nodesBodies,
        boost::shared_ptr<DynamicsSystemType> pDynSys
    ) : m_pSceneMgr(pSceneMgr),  m_rSceneNodeSimBodies(nodesSimBodies), m_rSceneNodeBodies(nodesBodies) , SceneParser<TConfig>(pDynSys)
    {
        ASSERTMSG(baseFrame != NULL, "Pointer is NULL");
        m_BaseFrame = baseFrame;
        this->m_bParseDynamics = true;
        this->m_SimBodies = 0;
    }

    SceneParserOgre(
        Ogre::SceneNode * baseFrame,
        boost::shared_ptr<Ogre::SceneManager> pSceneMgr,
        std::vector<Ogre::SceneNode*> &nodesSimBodies,
        std::vector<Ogre::SceneNode*> &nodesBodies
    )
        : m_pSceneMgr(pSceneMgr),  m_rSceneNodeSimBodies(nodesSimBodies), m_rSceneNodeBodies(nodesBodies) {
        ASSERTMSG(baseFrame != NULL, "Pointer is NULL");
        m_BaseFrame = baseFrame;
        this->m_bParseDynamics = false;
        this->m_SimBodies = 0;
    }

    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void processMesh( ticpp::Node * meshNode ) {

        static int nodeCounter = 0;
        static int entityCounter = 0;

        boost::filesystem::path meshName = meshNode->ToElement()->GetAttribute<std::string>("file");

        bool scaleLikeGeometry = false;
        Vector3 scale;
        if(meshNode->ToElement()->HasAttribute("scaleLikeGeometry")) {
            if(!Utilities::stringToType<bool>(scaleLikeGeometry, meshNode->ToElement()->GetAttribute("scaleLikeGeometry"))) {
                throw ticpp::Exception("String conversion of scale in processMesh: scaleWithGeometry failed");
            }
        } else {

            if(!Utilities::stringToVector3<PREC>(scale, meshNode->ToElement()->GetAttribute("scale"))) {
                throw ticpp::Exception("String conversion of scale in processMesh: scale failed");
            }
        }


        ticpp::Element * rendering =  meshNode->FirstChildElement("Rendering", false);
        bool attachAxis = false;
        double axesSize = 1;
        bool shadowsEnabled = true;
        if(rendering) {
            bool attachAxis = false;
            if(rendering->HasAttribute("attachAxis")) {
                if(!Utilities::stringToType<bool>(attachAxis, rendering->GetAttribute("attachAxis"))) {
                    throw ticpp::Exception("String conversion of in processMesh: attachAxis failed");
                }
            }

            double axesSize = 1;
            if(rendering->HasAttribute("axesSize")) {
                if(!Utilities::stringToType<double>(axesSize, rendering->GetAttribute("axesSize"))) {
                    throw ticpp::Exception("String conversion of in processMesh: axesSize failed");
                }
            }


            if(rendering->HasAttribute("shadowsEnabled")) {
                if(!Utilities::stringToType<bool>(shadowsEnabled, rendering->GetAttribute("shadowsEnabled"))) {
                    throw ticpp::Exception("String conversion of in processMesh: shadowsEnabled failed");
                }
            }
        };

        std::string type = meshNode->ToElement()->GetAttribute("type");

        if( type == "permutate" || type == "uniform") {
            std::vector<std::string> m_materialList;
            // Iterate over all material, save in list!
            ticpp::Iterator< ticpp::Element > material("Material");
            for ( material = material.begin( meshNode ); material != material.end(); material++ ) {
                m_materialList.push_back(material->GetAttribute<std::string>("name"));
            }
            if(m_materialList.empty()) {
                throw ticpp::Exception("No Material Node found in Mesh!");
            }



            std::stringstream entity_name,node_name;

            for(int i=0; i<this->m_bodyList.size(); i++) {
                entity_name.str("");
                node_name.str("");
                entity_name << meshName.filename().string() << std::string("Entity");
                node_name << meshName.filename().string() << std::string("Node");
                entity_name << entityCounter;
                node_name<< nodeCounter;
                //cout << entity_name.str() <<endl;
                Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(), meshName.string() );
                ent->setCastShadows(shadowsEnabled);
                Ogre::SceneNode* sceneNode = m_BaseFrame->createChildSceneNode(node_name.str());
                Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();
                if(scaleLikeGeometry) {
                    sceneNodeScale->setScale(this->m_bodyListScales[i](0),this->m_bodyListScales[i](1),this->m_bodyListScales[i](2));
                } else {
                    sceneNodeScale->setScale(scale(0),scale(1),scale(2));
                }
                sceneNodeScale->attachObject(ent);

                if(attachAxis) {
                    Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
                    Ogre::Entity* axisEnt = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt","axes.mesh" );
                    sceneNodeAxes->setScale(axesSize,axesSize,axesSize);
                    sceneNodeAxes->attachObject(axisEnt);
                }

                int matIdx = i % m_materialList.size();

                ent->setMaterialName(m_materialList[matIdx]);

                //Set initial condition
                sceneNode->setPosition(this->m_bodyList[i]->m_r_S(0),this->m_bodyList[i]->m_r_S(1),this->m_bodyList[i]->m_r_S(2));
                sceneNode->setOrientation(this->m_bodyList[i]->m_q_KI(0),this->m_bodyList[i]->m_q_KI(1),this->m_bodyList[i]->m_q_KI(2),this->m_bodyList[i]->m_q_KI(3));


                if( this->m_eBodiesState == RigidBodyType::SIMULATED) {
                    m_rSceneNodeSimBodies.push_back(sceneNode);
                } else if( this->m_eBodiesState == RigidBodyType::NOT_SIMULATED) {
                    m_rSceneNodeBodies.push_back(sceneNode);
                }

                nodeCounter++;
                entityCounter++;
            }



        } else {
            throw ticpp::Exception("The attribute 'type' '" + type + std::string("' of 'processMesh' has no implementation in the parser"));
        }
    }


protected:
    boost::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
    Ogre::SceneNode * m_BaseFrame;
    std::vector<Ogre::SceneNode*>	&m_rSceneNodeSimBodies;
    std::vector<Ogre::SceneNode*>	&m_rSceneNodeBodies;

};


#endif

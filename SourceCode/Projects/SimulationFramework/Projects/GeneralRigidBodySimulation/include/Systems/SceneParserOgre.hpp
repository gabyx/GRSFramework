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
    ) : m_pSceneMgr(pSceneMgr),  m_rSceneNodeSimBodies(nodesSimBodies), m_rSceneNodeBodies(nodesBodies) , SceneParser<TConfig>(pDynSys) {
        ASSERTMSG(baseFrame != NULL, "Pointer is NULL");
        m_BaseFrame = baseFrame;
        this->m_bParseDynamics = true;
        this->m_nSimBodies = 0;
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
        this->m_nSimBodies = 0;
    }

protected:

    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void processOtherOptions(const ticpp::Node * rootNode) {
        LOG(this->m_pSimulationLog,"Process MPISettings..."<<std::endl;);
        processMPISettings(rootNode);


        processSceneVisualizationSettings(rootNode);

    }

    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void processMesh(ticpp::Node * meshNode ) {

        static int nodeCounter = 0;
        static int entityCounter = 0;

        boost::filesystem::path meshName = meshNode->ToElement()->GetAttribute<std::string>("file");

        bool scaleLikeGeometry = false;
        Vector3 scale;
        if(meshNode->ToElement()->HasAttribute("scaleLikeGeometry")) {
            if(!Utilities::stringToType<bool>(scaleLikeGeometry, meshNode->ToElement()->GetAttribute("scaleLikeGeometry"))) {
                throw ticpp::Exception("String conversion of scale in processMesh: scaleWithGeometry failed");
            }
        }

        if(!scaleLikeGeometry){
            if(!Utilities::stringToVector3<PREC>(scale, meshNode->ToElement()->GetAttribute("scale"))) {
                throw ticpp::Exception("String conversion of scale in processMesh: scale failed");
            }
        }


        ticpp::Element * rendering =  meshNode->FirstChildElement("Rendering", false);
        bool attachAxis = false;
        double axesSize = 1;
        bool shadowsEnabled = true;
        if(rendering) {

            if(rendering->HasAttribute("attachAxis")) {
                if(!Utilities::stringToType<bool>(attachAxis, rendering->GetAttribute("attachAxis"))) {
                    throw ticpp::Exception("String conversion of in processMesh: attachAxis failed");
                }
            }
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
            LOG(this->m_pSimulationLog, " --> Add all Ogre Objects"<<std::endl);
            for(int i=0; i<this->m_bodyList.size(); i++) {
                entity_name.str("");
                node_name.str("");
                entity_name << meshName.filename().string() << std::string("Entity");
                node_name << meshName.filename().string() << std::string("Node");
                entity_name << entityCounter;
                node_name<< nodeCounter;

                //TODO m_pSceneMgr->createInstancedGeometry();


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

    void processMPISettings(const ticpp::Node * rootNode) {

        ticpp::Node * mpiSettings = rootNode->FirstChild("MPISettings",false);
        if(mpiSettings) {

            ticpp::Node *topo = mpiSettings->FirstChild("ProcessTopology",true);
            ticpp::Element *topoEl = topo->ToElement();

            std::string type = topoEl->GetAttribute("type");
            if(type=="grid") {
                Vector3 minPoint, maxPoint;
                if(!Utilities::stringToVector3<PREC>(minPoint,  topoEl->GetAttribute("minPoint"))) {
                    throw ticpp::Exception("String conversion in processMPISettings: minPoint failed");
                }
                if(!Utilities::stringToVector3<PREC>(maxPoint,  topoEl->GetAttribute("maxPoint"))) {
                    throw ticpp::Exception("String conversion in processMPISettings: maxPoint failed");
                }

                MyMatrix<unsigned int>::Vector3 dim;
                if(!Utilities::stringToVector3<unsigned int>(dim,  topoEl->GetAttribute("dimension"))) {
                    throw ticpp::Exception("String conversion in processMPISettings: dimension failed");
                }

                Vector3 extent;
                extent.array() = (maxPoint - minPoint);
                Vector3 center = 0.5*extent + minPoint;
                Vector3 dxyz;
                dxyz.array() = extent.array() / dim.array().template cast<PREC>();

                // Add MPI Visulaization
                ticpp::Element *mat =  topo->FirstChild("Visualization",true)->FirstChild("Material")->ToElement();
                std::string materialName = mat->GetAttribute("name");

                Ogre::SceneNode* mpiTopoRoot = m_BaseFrame->createChildSceneNode("MPIProcTopo");


                //Add subdivision in each direction
                Matrix33 I = Matrix33::Zero();
                I(1,0)=1;
                I(0,1)=1;
                I(2,2)=1;

                std::stringstream name;
                name.str("");
                name << "MPITopoSubGridManual-";
                Ogre::ManualObject *manual = m_pSceneMgr->createManualObject(name.str());

                manual->begin(materialName, Ogre::RenderOperation::OT_LINE_LIST);
                for(int k=0; k<dim(1)+1; k++) {
                    for(int l=0; l<dim(1)+1; l++) {
                        manual->position(minPoint(0),minPoint(1)+k*dxyz(1),minPoint(2)+l*dxyz(2));
                        manual->position(maxPoint(0),minPoint(1)+k*dxyz(1),minPoint(2)+l*dxyz(2));
                    }
                }

                for(int k=0; k<dim(0)+1; k++) {
                    for(int l=0; l<dim(0)+1; l++) {
                        manual->position(minPoint(0)+k*dxyz(0),minPoint(1)+l*dxyz(1),minPoint(2));
                        manual->position(minPoint(0)+k*dxyz(0),minPoint(1)+l*dxyz(1),maxPoint(2));
                    }
                }
                for(int k=0; k<dim(2)+1; k++) {
                    for(int l=0; l<dim(2)+1; l++) {
                        manual->position(minPoint(0)+l*dxyz(0),minPoint(1),minPoint(2)+k*dxyz(2));
                        manual->position(minPoint(0)+l*dxyz(0),maxPoint(1),minPoint(2)+k*dxyz(2));
                    }
                }
                manual->end();

                mpiTopoRoot->attachObject(manual);



//                    Ogre::SceneNode* mpiTopoRoot = m_BaseFrame->createChildSceneNode("MPIProcTopo");
//                    Ogre::SceneNode* mpiTopoBase = mpiTopoRoot->createChildSceneNode("MPIProcTopoBase");
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
//                            Ogre::SceneNode* mpiTopoSubdivs = mpiTopoRoot->createChildSceneNode(ent_name.str());
//                            ent_name.str(""); ent_name << "MPITopoSubGridEnt-" << i<<"-"<<k;
//                            Ogre::Entity *ent = m_pSceneMgr->createEntity(ent_name.str(), "Plane.mesh");
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



            } else {
                throw ticpp::Exception("String conversion in MPISettings:ProcessTopology:type failed: not a valid setting");
            }
        }

    }
    void processSceneVisualizationSettings(const ticpp::Node * rootNode) {

        ticpp::Node * sceneVisSettings = rootNode->FirstChild("SceneSettings",false)->FirstChild("Visualization",false);
        if(sceneVisSettings) {

            ticpp::Element *scaleNode = sceneVisSettings->FirstChild("SceneScale",false)->ToElement();
            if(scaleNode){
                double sceneScale = scaleNode->GetAttribute<double>("value");
                m_BaseFrame->setScale(Ogre::Vector3(1,1,1)*sceneScale);
            }
        }
    }

protected:
    boost::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
    Ogre::SceneNode * m_BaseFrame;
    std::vector<Ogre::SceneNode*>	&m_rSceneNodeSimBodies;
    std::vector<Ogre::SceneNode*>	&m_rSceneNodeBodies;

};


#endif


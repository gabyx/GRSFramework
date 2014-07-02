#ifndef SceneParserOgre_hpp
#define SceneParserOgre_hpp

#include <OGRE/Ogre.h>
#include "AxisObject.hpp"

#include "SceneParser.hpp"

#include "MakeCoordinateSystem.hpp"

class SceneParserOgre : public SceneParser {
public:

    DEFINE_CONFIG_TYPES

    // For simulation manager, playback manager doesnt set pDynSys, and does not parse Dynamics!
    SceneParserOgre(
        Ogre::SceneNode * baseFrame,
        std::shared_ptr<Ogre::SceneManager> pSceneMgr,
        std::vector<Ogre::SceneNode*> &nodesSimBodies,
        std::vector<Ogre::SceneNode*> &nodesBodies,
        std::shared_ptr<DynamicsSystemType> pDynSys = std::shared_ptr<DynamicsSystemType>(nullptr))
    : SceneParser(pDynSys),
      m_pSceneMgr(pSceneMgr),
      m_rSceneNodeSimBodies(nodesSimBodies),
      m_rSceneNodeBodies(nodesBodies),
      m_BaseFrame(baseFrame)
    {
        // Pointers are now set correctly to the corresponding maps in pDynSys
        ASSERTMSG(baseFrame != nullptr, "Pointer is nullptr");
    }

    // For simulation manager, playback manager doesnt set pDynSys, and does not parse Dynamics!
    SceneParserOgre(
        Ogre::SceneNode * baseFrame,
        std::shared_ptr<Ogre::SceneManager> pSceneMgr,
        std::vector<Ogre::SceneNode*> &nodesSimBodies,
        std::vector<Ogre::SceneNode*> &nodesBodies)
    : SceneParser(),
      m_pSceneMgr(pSceneMgr),
      m_rSceneNodeSimBodies(nodesSimBodies),
      m_rSceneNodeBodies(nodesBodies),
      m_BaseFrame(baseFrame)
    {
        // Pointers are now set correctly to the corresponding maps in pDynSys
        m_pGlobalGeometries = nullptr;
        m_pInitStates = nullptr;

        ASSERTMSG(baseFrame != nullptr, "Pointer is nullptr");
    }

protected:

    struct RenderSettings{
        bool attachAxis;
        double axesSize;
        bool shadowsEnabled;
    };

    void parseVisualization( ticpp::Node * visualizationNode) {


        ticpp::Node * meshNode = visualizationNode->FirstChild("Mesh",false);
        if(meshNode){
            parseMesh(meshNode);
        }
        else{
            ticpp::Node * planeNode = visualizationNode->FirstChild("Plane",false);
            parsePlane(planeNode);
        }


    };


    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void parseOtherOptions(const ticpp::Node * rootNode) {
        LOG(this->m_pSimulationLog,"---> Parse MPISettings..."<<std::endl;);
        parseMPISettings(rootNode);


        parseSceneVisualizationSettings(rootNode);

    }

    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void parseMesh(ticpp::Node * meshNode ) {

        static int nodeCounter = 0;
        static int entityCounter = 0;

        boost::filesystem::path meshName = meshNode->ToElement()->GetAttribute<std::string>("file");

        bool scaleLikeGeometry = false;
        Vector3 scale;
        if(meshNode->ToElement()->HasAttribute("scaleLikeGeometry")) {
            if(!Utilities::stringToType<bool>(scaleLikeGeometry, meshNode->ToElement()->GetAttribute("scaleLikeGeometry"))) {
                throw ticpp::Exception("---> String conversion in parseMesh: scaleWithGeometry failed");
            }
        }
        if(!scaleLikeGeometry){
            if(!Utilities::stringToVector3<PREC>(scale, meshNode->ToElement()->GetAttribute("scale"))) {
                throw ticpp::Exception("---> String conversion in parseMesh: scale failed");
            }
        }


        ticpp::Node * rendering =  meshNode->FirstChild("Rendering", false);
        RenderSettings renderSettings;
        parseRenderSettings(rendering, renderSettings);

        std::string type = meshNode->ToElement()->GetAttribute("type");
        if( type == "permutate" || type == "uniform") {
            m_materialList.clear();
            // Iterate over all material, save in list!
            ticpp::Iterator< ticpp::Element > material("Material");
            for ( material = material.begin( meshNode ); material != material.end(); material++ ) {
                m_materialList.push_back(material->GetAttribute<std::string>("name"));
            }
            if(m_materialList.empty()) {
                throw ticpp::Exception("---> No Material Node found in Mesh!");
            }
        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'parseMesh' has no implementation in the parser"));
        }


        std::stringstream entity_name,node_name;
        LOG(this->m_pSimulationLog, "---> Add all Ogre Mesh Objects"<<std::endl);

        unsigned int i; // Linear offset form the m_startIdGroup
        for(auto & b : m_bodyListGroup) {
            i = b.m_initState.m_id - m_startIdGroup;

            entity_name.str("");
            node_name.str("");
            entity_name << meshName.filename().string() << std::string("Entity");
            node_name << meshName.filename().string() << std::string("Node");
            entity_name << entityCounter;
            node_name<< nodeCounter;

            //TODO m_pSceneMgr->createInstancedGeometry();


            Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(), meshName.string() );
            ent->setCastShadows(renderSettings.shadowsEnabled);
            Ogre::SceneNode* sceneNode = m_BaseFrame->createChildSceneNode(node_name.str());
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();


            std::cout << b.m_scale.transpose() << std::endl;
            if(scaleLikeGeometry) {
                if(b.m_scale(0)<=0 || b.m_scale(1)<=0 || b.m_scale(2)<=0) {
                    throw ticpp::Exception("---> parseMesh:: Scale for Mesh: " + meshName.string() +"is zero or smaller!");
                }
                sceneNodeScale->setScale(b.m_scale(0),b.m_scale(1),b.m_scale(2));
            } else {
                if(scale(0)<=0 || scale(1)<=0 || scale(2)<=0) {
                    throw ticpp::Exception("---> parseMesh:: Scale for Mesh: " + meshName.string() + "is zero or smaller!");
                }
                sceneNodeScale->setScale(scale(0),scale(1),scale(2));
            }
            sceneNodeScale->attachObject(ent);

            if(renderSettings.attachAxis) {
                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
                Ogre::Entity* axisEnt = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt","axes.mesh" );
                //Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() + "Axes",100);
                sceneNodeAxes->setScale(renderSettings.axesSize,renderSettings.axesSize,renderSettings.axesSize);
                sceneNodeAxes->attachObject(axisEnt);
            }


            int matIdx = i % m_materialList.size();

            ent->setMaterialName(m_materialList[matIdx]);

            //Set initial condition
            std::cout << b.m_body->m_r_S.transpose() << "," << b.m_body->m_q_KI.transpose() << std::endl;
            sceneNode->setPosition(b.m_body->m_r_S(0),b.m_body->m_r_S(1),b.m_body->m_r_S(2));
            sceneNode->setOrientation(b.m_body->m_q_KI(0),b.m_body->m_q_KI(1),b.m_body->m_q_KI(2),b.m_body->m_q_KI(3));


            if( this->m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
                m_rSceneNodeSimBodies.push_back(sceneNode);
            } else if( this->m_eBodiesState == RigidBodyType::BodyState::STATIC) {
                m_rSceneNodeBodies.push_back(sceneNode);
            }

            nodeCounter++;
            entityCounter++;
        }

    }

    void parsePlane(ticpp::Node * planeNode ) {

        static int nodeCounter = 0;
        static int entityCounter = 0;

        bool scaleLikeGeometry = false;
        Vector3 scale;
        if(planeNode->ToElement()->HasAttribute("scaleLikeGeometry")) {
            if(!Utilities::stringToType<bool>(scaleLikeGeometry, planeNode->ToElement()->GetAttribute("scaleLikeGeometry"))) {
                throw ticpp::Exception("---> String conversion in parsePlane: scaleWithGeometry failed");
            }
        }
        if(!scaleLikeGeometry){
            if(!Utilities::stringToVector3<PREC>(scale, planeNode->ToElement()->GetAttribute("scale"))) {
                throw ticpp::Exception("---> String conversion  in parsePlane: scale failed");
            }
        }


        Vector2 subDivs = Vector2::Ones();
        if(planeNode->ToElement()->HasAttribute("subDivisions")) {
            if(!Utilities::stringToVector2<PREC>(subDivs, planeNode->ToElement()->GetAttribute("subDivisions"))) {
                throw ticpp::Exception("---> String conversion in parsePlane: subDivisions failed");
            }
        }

        Vector3 normal; normal(0)=0; normal(1)=0; normal(2)=1;
        if(planeNode->ToElement()->HasAttribute("normal")) {
            if(!Utilities::stringToVector3<PREC>(normal, planeNode->ToElement()->GetAttribute("normal"))) {
                throw ticpp::Exception("---> String conversion in parsePlane: normal failed");
            }
        }

        PREC dist=0;
        if(planeNode->ToElement()->HasAttribute("distance")) {
            if(!Utilities::stringToType<PREC>(dist, planeNode->ToElement()->GetAttribute("distance"))) {
                throw ticpp::Exception("---> String conversion in parsePlane: distance failed");
            }
        }

        Vector2 tile; tile(0)=1; tile(1)=1;
        if(planeNode->ToElement()->HasAttribute("tileTexture")) {
            if(!Utilities::stringToVector2<PREC>(tile, planeNode->ToElement()->GetAttribute("tileTexture"))) {
                throw ticpp::Exception("---> String conversion in parsePlane: tileTexture failed");
            }
        }



        ticpp::Node * rendering =  planeNode->FirstChild("Rendering", false);
        RenderSettings renderSettings;
        parseRenderSettings(rendering, renderSettings);

        //Distribution Type
        std::string type = planeNode->ToElement()->GetAttribute("type");
        if( type == "permutate" || type == "uniform") {
            m_materialList.clear();
            // Iterate over all material, save in list!
            ticpp::Iterator< ticpp::Element > material("Material");
            for ( material = material.begin( planeNode ); material != material.end(); material++ ) {
                m_materialList.push_back(material->GetAttribute<std::string>("name"));
            }
            if(m_materialList.empty()) {
                throw ticpp::Exception("---> No Material Node found in Plane!");
            }
        } else {
            throw ticpp::Exception("---> The attribute 'type' '" + type + std::string("' of 'parsePlane' has no implementation in the parser"));
        }


        std::stringstream entity_name,node_name,plane_name;
        LOG(this->m_pSimulationLog, "---> Add all Ogre: Plane Objects"<<std::endl);

        unsigned int i; // linear offset from m_startIdGroup
        for(auto & b : m_bodyListGroup) {
            i = b.m_initState.m_id - m_startIdGroup;

            entity_name.str("");
            node_name.str("");
            entity_name << "OgrePlane" << std::string("Entity");
            node_name << "OgrePlane" << std::string("Node");
            plane_name <<  "OgrePlane";
            entity_name << entityCounter;
            node_name<< nodeCounter;
            plane_name << nodeCounter;
            //TODO m_pSceneMgr->createInstancedGeometry();

            //Make mesh

            Ogre::Plane plane;
            plane.normal = Ogre::Vector3(normal(0),normal(1),normal(2));
            plane.d = dist;

            // Do some calculation becaus eOgre nees a correct UpVector ...
            Vector3 v1,v2;
            makeCoordinateSystem<PREC>(normal,v1,v2);

            Ogre::MeshManager::getSingleton().createPlane(plane_name.str(),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
            scale(0),scale(1),subDivs(0),subDivs(1),true,1,tile(0),tile(1),Ogre::Vector3(v1(0),v1(1),v1(2)));

            Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(),plane_name.str() );
            ent->setCastShadows(renderSettings.shadowsEnabled);

             Ogre::SceneNode* sceneNode = m_BaseFrame->createChildSceneNode(node_name.str());
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();
             if(scaleLikeGeometry) {
                sceneNodeScale->setScale(b.m_scale(0),b.m_scale(1),b.m_scale(2));
            } else {
                sceneNodeScale->setScale(scale(0),scale(1),scale(2));
            }
            sceneNodeScale->attachObject(ent);

            if(renderSettings.attachAxis) {
                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
                Ogre::Entity* axisEnt = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt","axes.mesh" );
                //Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() + "Axes",100);
                sceneNodeAxes->setScale(renderSettings.axesSize,renderSettings.axesSize,renderSettings.axesSize);
                sceneNodeAxes->attachObject(axisEnt);
            }

            int matIdx = i % m_materialList.size();

            ent->setMaterialName(m_materialList[matIdx]);

            //Set initial condition
            sceneNode->setPosition(b.m_body->m_r_S(0),b.m_body->m_r_S(1),b.m_body->m_r_S(2));
            sceneNode->setOrientation(b.m_body->m_q_KI(0),b.m_body->m_q_KI(1),b.m_body->m_q_KI(2),b.m_body->m_q_KI(3));


            if( this->m_eBodiesState == RigidBodyType::BodyState::SIMULATED) {
                m_rSceneNodeSimBodies.push_back(sceneNode);
            } else if( this->m_eBodiesState == RigidBodyType::BodyState::STATIC) {
                m_rSceneNodeBodies.push_back(sceneNode);
            }

            nodeCounter++;
            entityCounter++;
        }

    }



    void parseRenderSettings(ticpp::Node * rendering, RenderSettings & settings){

        settings.attachAxis = false;
        settings.axesSize = 1;
        settings.shadowsEnabled = true;
        if(rendering) {
            ticpp::Element* renderEl = rendering->ToElement();
            if(renderEl->HasAttribute("attachAxis")) {
                if(!Utilities::stringToType<bool>(settings.attachAxis, renderEl->GetAttribute("attachAxis"))) {
                    throw ticpp::Exception("---> String conversion of in parseMesh: attachAxis failed");
                }
            }
            if(renderEl->HasAttribute("axesSize")) {
                if(!Utilities::stringToType<double>(settings.axesSize, renderEl->GetAttribute("axesSize"))) {
                    throw ticpp::Exception("---> String conversion of in parseMesh: axesSize failed");
                }
            }


            if(renderEl->HasAttribute("shadowsEnabled")) {
                if(!Utilities::stringToType<bool>(settings.shadowsEnabled, renderEl->GetAttribute("shadowsEnabled"))) {
                    throw ticpp::Exception("---> String conversion of in parseMesh: shadowsEnabled failed");
                }
            }
        }

    }

    void parseMPISettings(const ticpp::Node * rootNode) {

        ticpp::Node * mpiSettings = rootNode->FirstChild("MPISettings",false);
        if(mpiSettings) {

            ticpp::Node *topo = mpiSettings->FirstChild("ProcessTopology",true);
            ticpp::Element *topoEl = topo->ToElement();

            std::string type = topoEl->GetAttribute("type");
            if(type=="grid") {
                Vector3 minPoint, maxPoint;
                if(!Utilities::stringToVector3<PREC>(minPoint,  topoEl->GetAttribute("minPoint"))) {
                    throw ticpp::Exception("---> String conversion in parseMPISettings: minPoint failed");
                }
                if(!Utilities::stringToVector3<PREC>(maxPoint,  topoEl->GetAttribute("maxPoint"))) {
                    throw ticpp::Exception("---> String conversion in parseMPISettings: maxPoint failed");
                }

                MyMatrix<unsigned int>::Vector3 dim;
                if(!Utilities::stringToVector3<unsigned int>(dim,  topoEl->GetAttribute("dimension"))) {
                    throw ticpp::Exception("---> String conversion in parseMPISettings: dimension failed");
                }

                Vector3 extent;
                extent.array() = (maxPoint - minPoint);
                Vector3 center = 0.5*extent + minPoint;
                Vector3 dxyz;
                dxyz.array() = extent.array() / dim.array().cast<PREC>();

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
                    for(int l=0; l<dim(2)+1; l++) {
                        manual->position(minPoint(0),minPoint(1)+k*dxyz(1),minPoint(2)+l*dxyz(2));
                        manual->position(maxPoint(0),minPoint(1)+k*dxyz(1),minPoint(2)+l*dxyz(2));
                    }
                }

                for(int k=0; k<dim(0)+1; k++) {
                    for(int l=0; l<dim(1)+1; l++) {
                        manual->position(minPoint(0)+k*dxyz(0),minPoint(1)+l*dxyz(1),minPoint(2));
                        manual->position(minPoint(0)+k*dxyz(0),minPoint(1)+l*dxyz(1),maxPoint(2));
                    }
                }
                for(int k=0; k<dim(2)+1; k++) {
                    for(int l=0; l<dim(0)+1; l++) {
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
                throw ticpp::Exception("---> String conversion in MPISettings:parseTopology:type failed: not a valid setting");
            }
        }

    }
    void parseSceneVisualizationSettings(const ticpp::Node * rootNode) {

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

    std::vector<std::string> m_materialList;

    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
    Ogre::SceneNode * m_BaseFrame;
    std::vector<Ogre::SceneNode*>	&m_rSceneNodeSimBodies;
    std::vector<Ogre::SceneNode*>	&m_rSceneNodeBodies;

};


#endif


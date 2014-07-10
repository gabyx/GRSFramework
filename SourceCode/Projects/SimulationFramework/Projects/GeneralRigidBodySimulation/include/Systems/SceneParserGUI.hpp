#ifndef SceneParserOgre_hpp
#define SceneParserOgre_hpp

#include <OGRE/Ogre.h>
#include "AxisObject.hpp"

#include "SceneParser.hpp"

#include "MakeCoordinateSystem.hpp"

namespace ParserModules{
    template<typename TParser>
    class VisModule {
    private:
        DEFINE_PARSER_CONFIG_TYPES_FOR_MODULE

        DEFINE_MATRIX_TYPES

        using BodyListType = typename BodyModuleType::BodyListType;
        using BodyMode = typename DynamicsSystemType::RigidBodyType::BodyMode;

        BodyMode m_mode;
        BodyListType * m_bodyListGroup = nullptr;
        RigidBodyIdType m_startIdGroup;

        using RigidBodyGraphicsType = typename DynamicsSystemType::RigidBodyGraphicsType;
        using RigidBodyGraphicsContType = typename DynamicsSystemType::RigidBodyGraphicsContType;
        RigidBodyGraphicsContType * m_pSimBodies;
        RigidBodyGraphicsContType * m_pBodies;

        Ogre::SceneManager * m_pSceneMgr;
        Ogre::SceneNode * m_pBaseNode;
        Ogre::SceneNode * m_pBodiesNode;

        ParserType * m_parser;

        struct RenderSettings{
            bool attachAxis =false;
            double axesSize = 1;
            bool shadowsEnabled = true;
        };

        std::vector<std::string> m_materialList;

    public:



        VisModule(ParserType * p,
                  RigidBodyGraphicsContType * pSimBodies,
                  RigidBodyGraphicsContType * pBodies,
                  Ogre::SceneNode * pBaseNode,
                  Ogre::SceneNode * pBodiesNode,
                  Ogre::SceneManager * pSceneMgr)
                  : m_parser(p), m_pSimBodies(pSimBodies), m_pBodies(pBodies), m_pBaseNode(pBaseNode), m_pBodiesNode(pBodiesNode), m_pSceneMgr(pSceneMgr)
        {
            ASSERTMSG(m_pSceneMgr && m_pBodiesNode, "these should not be zero!")

        };

        void parse(XMLNodeType vis, BodyListType * bodyList, RigidBodyIdType startId, BodyMode mode) {
            m_mode = mode;
            m_startIdGroup = startId;
            m_bodyListGroup = bodyList;

            LOGSCLEVEL1(m_parser->m_pSimulationLog, "==== VisModule: parsing (BodyVisualization)  ======================"<<std::endl;)
            XMLNodeType node = vis.child("Mesh");
            if(node){
                parseMesh(node);
            }
            else{
                node = vis.child("Plane");
                parsePlane(node);
            }
            LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)

        }

        void parseSceneSettingsPost(XMLNodeType sceneSettings) {

            LOGSCLEVEL1(m_parser->m_pSimulationLog, "==== VisModule: parsing (SceneSettingsPost, MPI/Visualization)  ===="<<std::endl;)

            parseMPISettings(sceneSettings);
            parseSceneVisualizationSettings(sceneSettings);

            LOGSCLEVEL1(m_parser->m_pSimulationLog, "==================================================================="<<std::endl;)

        }

    private:

    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
    void parseMesh(XMLNodeType  meshNode ) {
        XMLAttributeType att;
        static int nodeCounter = 0;
        static int entityCounter = 0;

        GET_XMLATTRIBUTE_CHECK(att,"file",meshNode);
        boost::filesystem::path meshName = att.value();

        bool scaleLikeGeometry = false;
        Vector3 scale;
        att = meshNode.attribute("scaleLikeGeometry");
        if(att) {
            if(!Utilities::stringToType<bool>(scaleLikeGeometry, att.value())) {
                THROWEXCEPTION("---> String conversion in parseMesh: scaleWithGeometry failed");
            }
        }
        if(!scaleLikeGeometry){
            if(!Utilities::stringToVector3<PREC>(scale, meshNode.attribute("scale").value() )) {
                THROWEXCEPTION("---> String conversion in parseMesh: scale failed");
            }
        }


        XMLNodeType  rendering = meshNode.child("Rendering");
        RenderSettings renderSettings;
        if(rendering){
            parseRenderSettings(rendering, renderSettings);
        }

        parseMaterials(meshNode);


        std::stringstream entity_name,node_name;
        LOG(m_parser->m_pSimulationLog, "---> Add all Ogre Mesh Objects"<<std::endl);

        unsigned int i; // Linear offset form the m_startIdGroup
        for(auto & b : *m_bodyListGroup) {
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
            Ogre::SceneNode* sceneNode = m_pBodiesNode->createChildSceneNode(node_name.str());
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();

            RigidBodyGraphicsType rigidBodyGraphics(sceneNode, b.m_initState.m_id);

            //std::cout << b.m_scale.transpose() << std::endl;
            if(scaleLikeGeometry) {
                if(b.m_scale(0)<=0 || b.m_scale(1)<=0 || b.m_scale(2)<=0) {
                    THROWEXCEPTION("---> parseMesh:: Scale for Mesh: " + meshName.string() +"is zero or smaller!");
                }
                sceneNodeScale->setScale(b.m_scale(0),b.m_scale(1),b.m_scale(2));
            } else {
                if(scale(0)<=0 || scale(1)<=0 || scale(2)<=0) {
                    THROWEXCEPTION("---> parseMesh:: Scale for Mesh: " + meshName.string() + "is zero or smaller!");
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
            rigidBodyGraphics.applyBodyState( b.m_initState );


            if( m_mode == BodyMode::SIMULATED) {
                m_pSimBodies->addBody(rigidBodyGraphics);
            } else if( m_mode == BodyMode::STATIC) {
                m_pBodies->addBody(rigidBodyGraphics);
            }

            nodeCounter++;
            entityCounter++;
        }

    }

    void parsePlane(XMLNodeType  planeNode ) {
        XMLAttributeType att;
        static int nodeCounter = 0;
        static int entityCounter = 0;

        bool scaleLikeGeometry = false;
        Vector3 scale;
        att = planeNode.attribute("scaleLikeGeometry");
        if(att) {
            if(!Utilities::stringToType<bool>(scaleLikeGeometry, att.value())) {
                THROWEXCEPTION("---> String conversion in parseMesh: scaleWithGeometry failed");
            }
        }
        if(!scaleLikeGeometry){
            if(!Utilities::stringToVector3<PREC>(scale, planeNode.attribute("scale").value() )) {
                THROWEXCEPTION("---> String conversion in parseMesh: scale failed");
            }
        }


        Vector2 subDivs = Vector2::Ones();
        att = planeNode.attribute("subDivisions");
        if(att) {
            if(!Utilities::stringToVector2<PREC>(subDivs, att.value())) {
                THROWEXCEPTION("---> String conversion in parsePlane: subDivisions failed");
            }
        }

        Vector3 normal; normal(0)=0; normal(1)=0; normal(2)=1;
        att = planeNode.attribute("normal");
        if(att) {
            if(!Utilities::stringToVector3<PREC>(normal, att.value())) {
                THROWEXCEPTION("---> String conversion in parsePlane: normal failed");
            }
        }

        PREC dist=0;
        att = planeNode.attribute("distance");
        if(att) {
            if(!Utilities::stringToType<PREC>(dist, att.value())) {
                THROWEXCEPTION("---> String conversion in parsePlane: distance failed");
            }
        }

        Vector2 tile; tile(0)=1; tile(1)=1;
        att = planeNode.attribute("tileTexture");
        if(att) {
            if(!Utilities::stringToVector2<PREC>(tile, att.value())) {
                THROWEXCEPTION("---> String conversion in parsePlane: tileTexture failed");
            }
        }



        XMLNodeType  rendering = planeNode.child("Rendering");
        RenderSettings renderSettings;
        if(rendering){
            parseRenderSettings(rendering, renderSettings);
        }

        parseMaterials(planeNode);


        std::stringstream entity_name,node_name,plane_name;
        LOG(m_parser->m_pSimulationLog, "---> Add all Ogre: Plane Objects"<<std::endl);

        unsigned int i; // linear offset from m_startIdGroup
        for(auto & b : *m_bodyListGroup) {
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

            Ogre::SceneNode* sceneNode = m_pBodiesNode->createChildSceneNode(node_name.str());
            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();

            RigidBodyGraphicsType rigidBodyGraphics(sceneNode, b.m_initState.m_id);

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
            rigidBodyGraphics.applyBodyState( b.m_initState );


            if( m_mode == BodyMode::SIMULATED) {
                m_pSimBodies->addBody(rigidBodyGraphics);
            } else if( m_mode == BodyMode::STATIC) {
                m_pBodies->addBody(rigidBodyGraphics);
            }

            nodeCounter++;
            entityCounter++;
        }

    }

    void parseMaterials(XMLNodeType meshNode){
        std::string type = meshNode.attribute("type").value();
        if( type == "permutate" || type == "uniform") {
            m_materialList.clear();
            // Iterate over all material, save in list!
            for ( auto & n : meshNode.children("Material") ) {
                m_materialList.emplace_back(n.attribute("name").value());
            }
            if(m_materialList.empty()) {
                THROWEXCEPTION("---> No Material Node found in Mesh!");
            }
        } else {
            THROWEXCEPTION("---> The attribute 'type' '" << type << "' of 'parseMesh' has no implementation in the parser");
        }
    }

    void parseRenderSettings(XMLNodeType  rendering, RenderSettings & settings){
        XMLAttributeType att;
        att =rendering.attribute("attachAxis");
        if(att) {
            if(!Utilities::stringToType<bool>(settings.attachAxis, att.value() )) {
                THROWEXCEPTION("---> String conversion of in parseMesh: attachAxis failed");
            }
        }
        att =rendering.attribute("axesSize");
        if(att) {
            if(!Utilities::stringToType<double>(settings.axesSize, att.value())) {
                THROWEXCEPTION("---> String conversion of in parseMesh: axesSize failed");
            }
        }

        att = rendering.attribute("shadowsEnabled");
        if(att) {
            if(!Utilities::stringToType<bool>(settings.shadowsEnabled, att.value())) {
                THROWEXCEPTION("---> String conversion of in parseMesh: shadowsEnabled failed");
            }
        }
    }

    void parseMPISettings(XMLNodeType sceneSettings) {

        XMLNodeType  mpiSettings = sceneSettings.child("MPISettings");
        if(mpiSettings) {

            XMLNodeType topo;
            GET_XMLCHILDNODE_CHECK(topo,"ProcessTopology",mpiSettings );

            std::string type = topo.attribute("type").value();
            if(type=="grid") {
                Vector3 minPoint, maxPoint;
                if(!Utilities::stringToVector3<PREC>(minPoint,  topo.attribute("minPoint").value() )) {
                    THROWEXCEPTION("---> String conversion in parseMPISettings: minPoint failed");
                }
                if(!Utilities::stringToVector3<PREC>(maxPoint,  topo.attribute("maxPoint").value())) {
                    THROWEXCEPTION("---> String conversion in parseMPISettings: maxPoint failed");
                }

                MyMatrix<unsigned int>::Vector3 dim;
                if(!Utilities::stringToVector3<unsigned int>(dim,  topo.attribute("dimension").value())) {
                    THROWEXCEPTION("---> String conversion in parseMPISettings: dimension failed");
                }

                Vector3 extent;
                extent.array() = (maxPoint - minPoint);
                Vector3 center = 0.5*extent + minPoint;
                Vector3 dxyz;
                dxyz.array() = extent.array() / dim.array().cast<PREC>();

                // Add MPI Visulaization
                std::string materialName = topo.child("Visualization").child("Material").attribute("name").value();

                Ogre::SceneNode* mpiTopoRoot = m_pBaseNode->createChildSceneNode("MPIProcTopo");


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
                THROWEXCEPTION("---> String conversion in MPISettings:parseTopology:type failed: not a valid setting");
            }
        }

    }
    void parseSceneVisualizationSettings(XMLNodeType  sceneSettings) {

        XMLNodeType  sceneVisSettings = sceneSettings.child("Visualization");
        if(sceneVisSettings) {

            XMLNodeType scaleNode = sceneVisSettings.child("SceneScale");
            if(scaleNode){
                double sceneScale;
                if(!Utilities::stringToType<double>(sceneScale,  scaleNode.attribute("value").value())) {
                    THROWEXCEPTION("---> String conversion in SceneScale: value failed");
                }
                m_pBaseNode->setScale(Ogre::Vector3(1,1,1)*sceneScale);
            }
        }
    }

    };
};

/** These module types are defined when there is no derivation from scene parser */
template<typename TSceneParser>
struct SceneParserGUIModuleTraits{
    using SettingsModuleType         = ParserModules::SettingsModule<TSceneParser>;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<TSceneParser>;
    using ContactParamModuleType     = ParserModules::ContactParamModule<TSceneParser>;
    using InitStatesModuleType       = ParserModules::InitStatesModule<TSceneParser> ;

    using BodyModuleType             = ParserModules::BodyModule< TSceneParser > ;
    using GeometryModuleType         = ParserModules::GeometryModule<TSceneParser>;

    using VisModuleType              = ParserModules::VisModule<TSceneParser>;
};

template<typename TDynamicsSystem>
class SceneParserGUI: public SceneParser<TDynamicsSystem, SceneParserGUIModuleTraits, SceneParserGUI<TDynamicsSystem> > {
private:
    using BaseType = SceneParser<TDynamicsSystem, SceneParserGUIModuleTraits, SceneParserGUI<TDynamicsSystem> >;
public:
    using DynamicsSystemType = TDynamicsSystem;

    DEFINE_MODULETYPES_AND_FRIENDS(BaseType);
    DEFINE_PARSER_CONFIG_TYPES_OF_BASE(BaseType);


public:


    SceneParserGUI( std::shared_ptr<DynamicsSystemType> pDynSys): BaseType(pDynSys) {


    }

    void dooo(){
        this->m_pVisModule = std::unique_ptr<VisModuleType>(new VisModuleType(this));
        this->m_pVisModule->parse(XMLNodeType());
    }

};

//
//class SceneParserGUI : public SceneParser {
//public:
//
//    DEFINE_CONFIG_TYPES
//
//    // For simulation manager, playback manager doesnt set pDynSys, and does not parse Dynamics!
//    SceneParserGUI(
//        Ogre::SceneNode * baseFrame,
//        std::shared_ptr<Ogre::SceneManager> pSceneMgr,
//        std::vector<Ogre::SceneNode*> &nodesSimBodies,
//        std::vector<Ogre::SceneNode*> &nodesBodies,
//        std::shared_ptr<DynamicsSystemType> pDynSys = std::shared_ptr<DynamicsSystemType>(nullptr))
//    : SceneParser(pDynSys),
//      m_pSceneMgr(pSceneMgr),
//      m_rSceneNodeSimBodies(nodesSimBodies),
//      m_rSceneNodeBodies(nodesBodies),
//      m_BaseFrame(baseFrame)
//    {
//        // Pointers are now set correctly to the corresponding maps in pDynSys
//        ASSERTMSG(baseFrame != nullptr, "Pointer is nullptr");
//    }
//
//    // For simulation manager, playback manager doesnt set pDynSys, and does not parse Dynamics!
//    SceneParserGUI(
//        Ogre::SceneNode * baseFrame,
//        std::shared_ptr<Ogre::SceneManager> pSceneMgr,
//        std::vector<Ogre::SceneNode*> &nodesSimBodies,
//        std::vector<Ogre::SceneNode*> &nodesBodies)
//    : SceneParser(),
//      m_pSceneMgr(pSceneMgr),
//      m_rSceneNodeSimBodies(nodesSimBodies),
//      m_rSceneNodeBodies(nodesBodies),
//      m_BaseFrame(baseFrame)
//    {
//        // Pointers are now set correctly to the corresponding maps in pDynSys
//        m_pGlobalGeometries = nullptr;
//        m_pInitStates = nullptr;
//
//        ASSERTMSG(baseFrame != nullptr, "Pointer is nullptr");
//    }
//
//protected:
//
//    struct RenderSettings{
//        bool attachAxis;
//        double axesSize;
//        bool shadowsEnabled;
//    };
//
//    void parseVisualization( XMLNodeType  visualizationNode) {
//
//
//        XMLNodeType  meshNode = visualizationNode.child("Mesh",false);
//        if(meshNode){
//            parseMesh(meshNode);
//        }
//        else{
//            XMLNodeType  planeNode = visualizationNode.child("Plane",false);
//            parsePlane(planeNode);
//        }
//
//
//    };
//
//
//    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
//    void parseOtherOptions(const XMLNodeType  rootNode) {
//        LOG(this->m_pSimulationLog,"---> Parse MPISettings..."<<std::endl;);
//        parseMPISettings(rootNode);
//
//
//        parseSceneVisualizationSettings(rootNode);
//
//    }
//
//    // Virtual function in SceneParser!, this function adds all objects to Ogre related objects!
//    void parseMesh(XMLNodeType  meshNode ) {
//
//        static int nodeCounter = 0;
//        static int entityCounter = 0;
//
//        boost::filesystem::path meshName = meshNode.attribute<std::string>("file");
//
//        bool scaleLikeGeometry = false;
//        Vector3 scale;
//        if(meshNode.attribute("scaleLikeGeometry")) {
//            if(!Utilities::stringToType<bool>(scaleLikeGeometry, meshNode.attribute("scaleLikeGeometry"))) {
//                THROWEXCEPTION("---> String conversion in parseMesh: scaleWithGeometry failed");
//            }
//        }
//        if(!scaleLikeGeometry){
//            if(!Utilities::stringToVector3<PREC>(scale, meshNode.attribute("scale"))) {
//                THROWEXCEPTION("---> String conversion in parseMesh: scale failed");
//            }
//        }
//
//
//        XMLNodeType  rendering =  meshNode.child("Rendering", false);
//        RenderSettings renderSettings;
//        parseRenderSettings(rendering, renderSettings);
//
//        std::string type = meshNode.attribute("type");
//        if( type == "permutate" || type == "uniform") {
//            m_materialList.clear();
//            // Iterate over all material, save in list!
//            ticpp::Iterator< ticpp::Element > material("Material");
//            for ( material = material.begin( meshNode ); material != material.end(); material++ ) {
//                m_materialList.push_back(material.attribute<std::string>("name"));
//            }
//            if(m_materialList.empty()) {
//                THROWEXCEPTION("---> No Material Node found in Mesh!");
//            }
//        } else {
//            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'parseMesh' has no implementation in the parser"));
//        }
//
//
//        std::stringstream entity_name,node_name;
//        LOG(this->m_pSimulationLog, "---> Add all Ogre Mesh Objects"<<std::endl);
//
//        unsigned int i; // Linear offset form the m_startIdGroup
//        for(auto & b : m_bodyListGroup) {
//            i = b.m_initState.m_id - m_startIdGroup;
//
//            entity_name.str("");
//            node_name.str("");
//            entity_name << meshName.filename().string() << std::string("Entity");
//            node_name << meshName.filename().string() << std::string("Node");
//            entity_name << entityCounter;
//            node_name<< nodeCounter;
//
//            //TODO m_pSceneMgr->createInstancedGeometry();
//
//
//            Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(), meshName.string() );
//            ent->setCastShadows(renderSettings.shadowsEnabled);
//            Ogre::SceneNode* sceneNode = m_BaseFrame->createChildSceneNode(node_name.str());
//            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();
//
//
//            std::cout << b.m_scale.transpose() << std::endl;
//            if(scaleLikeGeometry) {
//                if(b.m_scale(0)<=0 || b.m_scale(1)<=0 || b.m_scale(2)<=0) {
//                    THROWEXCEPTION("---> parseMesh:: Scale for Mesh: " + meshName.string() +"is zero or smaller!");
//                }
//                sceneNodeScale->setScale(b.m_scale(0),b.m_scale(1),b.m_scale(2));
//            } else {
//                if(scale(0)<=0 || scale(1)<=0 || scale(2)<=0) {
//                    THROWEXCEPTION("---> parseMesh:: Scale for Mesh: " + meshName.string() + "is zero or smaller!");
//                }
//                sceneNodeScale->setScale(scale(0),scale(1),scale(2));
//            }
//            sceneNodeScale->attachObject(ent);
//
//            if(renderSettings.attachAxis) {
//                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
//                Ogre::Entity* axisEnt = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt","axes.mesh" );
//                //Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() + "Axes",100);
//                sceneNodeAxes->setScale(renderSettings.axesSize,renderSettings.axesSize,renderSettings.axesSize);
//                sceneNodeAxes->attachObject(axisEnt);
//            }
//
//
//            int matIdx = i % m_materialList.size();
//
//            ent->setMaterialName(m_materialList[matIdx]);
//
//            //Set initial condition
//            std::cout << b.m_body->m_r_S.transpose() << "," << b.m_body->m_q_KI.transpose() << std::endl;
//            sceneNode->setPosition(b.m_body->m_r_S(0),b.m_body->m_r_S(1),b.m_body->m_r_S(2));
//            sceneNode->setOrientation(b.m_body->m_q_KI(0),b.m_body->m_q_KI(1),b.m_body->m_q_KI(2),b.m_body->m_q_KI(3));
//
//
//            if( this->m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
//                m_rSceneNodeSimBodies.push_back(sceneNode);
//            } else if( this->m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
//                m_rSceneNodeBodies.push_back(sceneNode);
//            }
//
//            nodeCounter++;
//            entityCounter++;
//        }
//
//    }
//
//    void parsePlane(XMLNodeType  planeNode ) {
//
//        static int nodeCounter = 0;
//        static int entityCounter = 0;
//
//        bool scaleLikeGeometry = false;
//        Vector3 scale;
//        if(planeNode.attribute("scaleLikeGeometry")) {
//            if(!Utilities::stringToType<bool>(scaleLikeGeometry, planeNode.attribute("scaleLikeGeometry"))) {
//                THROWEXCEPTION("---> String conversion in parsePlane: scaleWithGeometry failed");
//            }
//        }
//        if(!scaleLikeGeometry){
//            if(!Utilities::stringToVector3<PREC>(scale, planeNode.attribute("scale"))) {
//                THROWEXCEPTION("---> String conversion  in parsePlane: scale failed");
//            }
//        }
//
//
//        Vector2 subDivs = Vector2::Ones();
//        if(planeNode.attribute("subDivisions")) {
//            if(!Utilities::stringToVector2<PREC>(subDivs, planeNode.attribute("subDivisions"))) {
//                THROWEXCEPTION("---> String conversion in parsePlane: subDivisions failed");
//            }
//        }
//
//        Vector3 normal; normal(0)=0; normal(1)=0; normal(2)=1;
//        if(planeNode.attribute("normal")) {
//            if(!Utilities::stringToVector3<PREC>(normal, planeNode.attribute("normal"))) {
//                THROWEXCEPTION("---> String conversion in parsePlane: normal failed");
//            }
//        }
//
//        PREC dist=0;
//        if(planeNode.attribute("distance")) {
//            if(!Utilities::stringToType<PREC>(dist, planeNode.attribute("distance"))) {
//                THROWEXCEPTION("---> String conversion in parsePlane: distance failed");
//            }
//        }
//
//        Vector2 tile; tile(0)=1; tile(1)=1;
//        if(planeNode.attribute("tileTexture")) {
//            if(!Utilities::stringToVector2<PREC>(tile, planeNode.attribute("tileTexture"))) {
//                THROWEXCEPTION("---> String conversion in parsePlane: tileTexture failed");
//            }
//        }
//
//
//
//        XMLNodeType  rendering =  planeNode.child("Rendering", false);
//        RenderSettings renderSettings;
//        parseRenderSettings(rendering, renderSettings);
//
//        //Distribution Type
//        std::string type = planeNode.attribute("type");
//        if( type == "permutate" || type == "uniform") {
//            m_materialList.clear();
//            // Iterate over all material, save in list!
//            ticpp::Iterator< ticpp::Element > material("Material");
//            for ( material = material.begin( planeNode ); material != material.end(); material++ ) {
//                m_materialList.push_back(material.attribute<std::string>("name"));
//            }
//            if(m_materialList.empty()) {
//                THROWEXCEPTION("---> No Material Node found in Plane!");
//            }
//        } else {
//            THROWEXCEPTION("---> The attribute 'type' '" + type + std::string("' of 'parsePlane' has no implementation in the parser"));
//        }
//
//
//        std::stringstream entity_name,node_name,plane_name;
//        LOG(this->m_pSimulationLog, "---> Add all Ogre: Plane Objects"<<std::endl);
//
//        unsigned int i; // linear offset from m_startIdGroup
//        for(auto & b : m_bodyListGroup) {
//            i = b.m_initState.m_id - m_startIdGroup;
//
//            entity_name.str("");
//            node_name.str("");
//            entity_name << "OgrePlane" << std::string("Entity");
//            node_name << "OgrePlane" << std::string("Node");
//            plane_name <<  "OgrePlane";
//            entity_name << entityCounter;
//            node_name<< nodeCounter;
//            plane_name << nodeCounter;
//            //TODO m_pSceneMgr->createInstancedGeometry();
//
//            //Make mesh
//
//            Ogre::Plane plane;
//            plane.normal = Ogre::Vector3(normal(0),normal(1),normal(2));
//            plane.d = dist;
//
//            // Do some calculation becaus eOgre nees a correct UpVector ...
//            Vector3 v1,v2;
//            makeCoordinateSystem<PREC>(normal,v1,v2);
//
//            Ogre::MeshManager::getSingleton().createPlane(plane_name.str(),
//            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
//            scale(0),scale(1),subDivs(0),subDivs(1),true,1,tile(0),tile(1),Ogre::Vector3(v1(0),v1(1),v1(2)));
//
//            Ogre::Entity* ent = m_pSceneMgr->createEntity(entity_name.str(),plane_name.str() );
//            ent->setCastShadows(renderSettings.shadowsEnabled);
//
//             Ogre::SceneNode* sceneNode = m_BaseFrame->createChildSceneNode(node_name.str());
//            Ogre::SceneNode* sceneNodeScale = sceneNode->createChildSceneNode();
//             if(scaleLikeGeometry) {
//                sceneNodeScale->setScale(b.m_scale(0),b.m_scale(1),b.m_scale(2));
//            } else {
//                sceneNodeScale->setScale(scale(0),scale(1),scale(2));
//            }
//            sceneNodeScale->attachObject(ent);
//
//            if(renderSettings.attachAxis) {
//                Ogre::SceneNode* sceneNodeAxes = sceneNode->createChildSceneNode(entity_name.str() + "Axes");
//                Ogre::Entity* axisEnt = m_pSceneMgr->createEntity(entity_name.str() + "AxesEnt","axes.mesh" );
//                //Ogre::MovableObject * axisEnt= AxisObject().createAxis(m_pSceneMgr.get(),entity_name.str() + "Axes",100);
//                sceneNodeAxes->setScale(renderSettings.axesSize,renderSettings.axesSize,renderSettings.axesSize);
//                sceneNodeAxes->attachObject(axisEnt);
//            }
//
//            int matIdx = i % m_materialList.size();
//
//            ent->setMaterialName(m_materialList[matIdx]);
//
//            //Set initial condition
//            sceneNode->setPosition(b.m_body->m_r_S(0),b.m_body->m_r_S(1),b.m_body->m_r_S(2));
//            sceneNode->setOrientation(b.m_body->m_q_KI(0),b.m_body->m_q_KI(1),b.m_body->m_q_KI(2),b.m_body->m_q_KI(3));
//
//
//            if( this->m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
//                m_rSceneNodeSimBodies.push_back(sceneNode);
//            } else if( this->m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
//                m_rSceneNodeBodies.push_back(sceneNode);
//            }
//
//            nodeCounter++;
//            entityCounter++;
//        }
//
//    }
//
//
//
//    void parseRenderSettings(XMLNodeType  rendering, RenderSettings & settings){
//
//        settings.attachAxis = false;
//        settings.axesSize = 1;
//        settings.shadowsEnabled = true;
//        if(rendering) {
//            ticpp::Element* renderEl = rendering->ToElement();
//            if(renderEl.attribute("attachAxis")) {
//                if(!Utilities::stringToType<bool>(settings.attachAxis, renderEl.attribute("attachAxis"))) {
//                    THROWEXCEPTION("---> String conversion of in parseMesh: attachAxis failed");
//                }
//            }
//            if(renderEl.attribute("axesSize")) {
//                if(!Utilities::stringToType<double>(settings.axesSize, renderEl.attribute("axesSize"))) {
//                    THROWEXCEPTION("---> String conversion of in parseMesh: axesSize failed");
//                }
//            }
//
//
//            if(renderEl.attribute("shadowsEnabled")) {
//                if(!Utilities::stringToType<bool>(settings.shadowsEnabled, renderEl.attribute("shadowsEnabled"))) {
//                    THROWEXCEPTION("---> String conversion of in parseMesh: shadowsEnabled failed");
//                }
//            }
//        }
//
//    }
//
//    void parseMPISettings(const XMLNodeType  rootNode) {
//
//        XMLNodeType  mpiSettings = rootNode.child("MPISettings",false);
//        if(mpiSettings) {
//
//            XMLNodeType topo = mpiSettings.child("ProcessTopology",true);
//            ticpp::Element *topoEl = topo->ToElement();
//
//            std::string type = topoEl.attribute("type");
//            if(type=="grid") {
//                Vector3 minPoint, maxPoint;
//                if(!Utilities::stringToVector3<PREC>(minPoint,  topoEl.attribute("minPoint"))) {
//                    THROWEXCEPTION("---> String conversion in parseMPISettings: minPoint failed");
//                }
//                if(!Utilities::stringToVector3<PREC>(maxPoint,  topoEl.attribute("maxPoint"))) {
//                    THROWEXCEPTION("---> String conversion in parseMPISettings: maxPoint failed");
//                }
//
//                MyMatrix<unsigned int>::Vector3 dim;
//                if(!Utilities::stringToVector3<unsigned int>(dim,  topoEl.attribute("dimension"))) {
//                    THROWEXCEPTION("---> String conversion in parseMPISettings: dimension failed");
//                }
//
//                Vector3 extent;
//                extent.array() = (maxPoint - minPoint);
//                Vector3 center = 0.5*extent + minPoint;
//                Vector3 dxyz;
//                dxyz.array() = extent.array() / dim.array().cast<PREC>();
//
//                // Add MPI Visulaization
//                ticpp::Element *mat =  topo.child("Visualization",true)->FirstChild("Material")->ToElement();
//                std::string materialName = mat.attribute("name");
//
//                Ogre::SceneNode* mpiTopoRoot = m_BaseFrame->createChildSceneNode("MPIProcTopo");
//
//
//                //Add subdivision in each direction
//                Matrix33 I = Matrix33::Zero();
//                I(1,0)=1;
//                I(0,1)=1;
//                I(2,2)=1;
//
//                std::stringstream name;
//                name.str("");
//                name << "MPITopoSubGridManual-";
//                Ogre::ManualObject *manual = m_pSceneMgr->createManualObject(name.str());
//
//                manual->begin(materialName, Ogre::RenderOperation::OT_LINE_LIST);
//                for(int k=0; k<dim(1)+1; k++) {
//                    for(int l=0; l<dim(2)+1; l++) {
//                        manual->position(minPoint(0),minPoint(1)+k*dxyz(1),minPoint(2)+l*dxyz(2));
//                        manual->position(maxPoint(0),minPoint(1)+k*dxyz(1),minPoint(2)+l*dxyz(2));
//                    }
//                }
//
//                for(int k=0; k<dim(0)+1; k++) {
//                    for(int l=0; l<dim(1)+1; l++) {
//                        manual->position(minPoint(0)+k*dxyz(0),minPoint(1)+l*dxyz(1),minPoint(2));
//                        manual->position(minPoint(0)+k*dxyz(0),minPoint(1)+l*dxyz(1),maxPoint(2));
//                    }
//                }
//                for(int k=0; k<dim(2)+1; k++) {
//                    for(int l=0; l<dim(0)+1; l++) {
//                        manual->position(minPoint(0)+l*dxyz(0),minPoint(1),minPoint(2)+k*dxyz(2));
//                        manual->position(minPoint(0)+l*dxyz(0),maxPoint(1),minPoint(2)+k*dxyz(2));
//                    }
//                }
//                manual->end();
//
//                mpiTopoRoot->attachObject(manual);
//
//
//
////                    Ogre::SceneNode* mpiTopoRoot = m_BaseFrame->createChildSceneNode("MPIProcTopo");
////                    Ogre::SceneNode* mpiTopoBase = mpiTopoRoot->createChildSceneNode("MPIProcTopoBase");
////                    Ogre::Entity *ent = m_pSceneMgr->createEntity("MPITopoBaseGrid", "Cube.mesh");
////                    ent->setMaterialName(materialName);
////                    mpiTopoBase->attachObject(ent);
////                    mpiTopoBase->setPosition(center(0),center(1),center(2));
////                    mpiTopoBase->setScale(0.5*extent(0),0.5*extent(1),0.5*extent(2));
////
////                    //Add subdivision in each direction
////                    Matrix33 I = Matrix33::Zero();
////                    I(1,0)=1; I(0,1)=1; I(2,2)=1;
////
////                    std::stringstream ent_name;
////                    for(int i=0;i<3;i++){
////                        Vector3 centerDivs = center;
////                        centerDivs(i) -=  0.5*extent(i);
////                        for(int k=1;k<dim(i);k++){
////                            ent_name.str(""); ent_name << "MPITopoSubGridNode-" << i<<"-"<<k;
////                            Ogre::SceneNode* mpiTopoSubdivs = mpiTopoRoot->createChildSceneNode(ent_name.str());
////                            ent_name.str(""); ent_name << "MPITopoSubGridEnt-" << i<<"-"<<k;
////                            Ogre::Entity *ent = m_pSceneMgr->createEntity(ent_name.str(), "Plane.mesh");
////                            ent->setMaterialName(materialName);
////                            mpiTopoSubdivs->attachObject(ent);
////
////
////                            mpiTopoSubdivs->rotate(Ogre::Vector3(I(0,i),I(1,i),I(2,i)),Ogre::Degree(90));
////                            if(i==0){
////                                mpiTopoSubdivs->scale(0.5*extent(2),0.5*extent(1),1);
////                            }else if(i==1){
////                                mpiTopoSubdivs->scale(0.5*extent(1),0.5*extent(2),1);
////                            }else{
////                                mpiTopoSubdivs->scale(0.5*extent(0),0.5*extent(1),1);
////                            }
////                            centerDivs(i) += dxyz(i);
////                            mpiTopoSubdivs->setPosition(centerDivs(0),centerDivs(1),centerDivs(2));
////
////                        }
////
////                    }
//
//
//
//            } else {
//                THROWEXCEPTION("---> String conversion in MPISettings:parseTopology:type failed: not a valid setting");
//            }
//        }
//
//    }
//    void parseSceneVisualizationSettings(const XMLNodeType  rootNode) {
//
//        XMLNodeType  sceneVisSettings = rootNode.child("SceneSettings",false)->FirstChild("Visualization",false);
//        if(sceneVisSettings) {
//
//            ticpp::Element *scaleNode = sceneVisSettings.child("SceneScale",false)->ToElement();
//            if(scaleNode){
//                double sceneScale = scaleNode.attribute<double>("value");
//                m_BaseFrame->setScale(Ogre::Vector3(1,1,1)*sceneScale);
//            }
//        }
//    }
//
//
//protected:
//
//    std::vector<std::string> m_materialList;
//
//    std::shared_ptr<Ogre::SceneManager> m_pSceneMgr;
//    Ogre::SceneNode * m_BaseFrame;
//    std::vector<Ogre::SceneNode*>	&m_rSceneNodeSimBodies;
//    std::vector<Ogre::SceneNode*>	&m_rSceneNodeBodies;
//
//};


#endif


#ifndef SceneParserMPI_hpp
#define SceneParserMPI_hpp

#include "SceneParser.hpp"

#include "MPITopologyBuilderSettings.hpp"

namespace ParserModules{

template<typename TParserTraits>
class SettingsModuleMPI : public SettingsModule<TParserTraits> {
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    using RecorderSettingsType = typename SettingsModule<TParserTraits>::RecorderSettingsType ;
    using TimeStepperSettingsType = typename SettingsModule<TParserTraits>::TimeStepperSettingsType     ;
    using InclusionSolverSettingsType = typename SettingsModule<TParserTraits>::InclusionSolverSettingsType ;



public:
    SettingsModuleMPI(ParserType * p, RecorderSettingsType * r, TimeStepperSettingsType * t, InclusionSolverSettingsType * i)
        :SettingsModule<TParserTraits>(p,r,t,i) {};

    void parseOtherOptions(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(this->m_pSimulationLog, "==== SettingsModuleMPI: parsing other options ====================="<<std::endl;)

        XMLNodeType incSet = sceneSettings.child("MPISettings").child("InclusionSolverSettings");
        CHECK_XMLNODE(incSet,"MPISettings/InclusionSolverSettings does not exist");
         // Process special Inclusion solver settings
        PREC splitNodeUpdateRatio;
        if(!Utilities::stringToType(splitNodeUpdateRatio,  incSet.attribute("splitNodeUpdateRatio").value())) {
                THROWEXCEPTION("---> String conversion in MPISettings::InclusionSolverSettings: splitNodeUpdateRatio failed");
        }
        if(splitNodeUpdateRatio <= 0){
            THROWEXCEPTION("---> MPISettings::InclusionSolverSettings: splitNodeUpdateRatio <= 0");
        }

        PREC convergenceCheckRatio;
        if(!Utilities::stringToType(convergenceCheckRatio,  incSet.attribute("convergenceCheckRatio").value())) {
                THROWEXCEPTION("---> String conversion in MPISettings::InclusionSolverSettings: convergenceCheckRatio failed");
        }
        if(convergenceCheckRatio <= 0){
            THROWEXCEPTION("---> MPISettings::InclusionSolverSettings: convergenceCheckRatio <= 0");
        }

        this->m_inclusionSettings->m_splitNodeUpdateRatio = splitNodeUpdateRatio;
        this->m_inclusionSettings->m_convergenceCheckRatio = convergenceCheckRatio;

        LOGSCLEVEL1(this->m_pSimulationLog, "==================================================================="<<std::endl;)
    }
};

template<typename TParserTraits>
class MPIModule {
private:
    DEFINE_PARSER_TYPE_TRAITS(TParserTraits )
    DEFINE_LAYOUT_CONFIG_TYPES

    LogType * m_pSimulationLog;

    using TopologyBuilderSettingsType = typename DynamicsSystemType::TopologyBuilderSettingsType;
    TopologyBuilderSettingsType * m_topoSettings;

public:
    MPIModule(ParserType * p, BodyModuleType * b, TopologyBuilderSettingsType * t)
    :m_pSimulationLog(p->getSimLog()), m_topoSettings(t) {}

    void parseSceneSettingsPost(XMLNodeType sceneSettings) {
        LOGSCLEVEL1(this->m_pSimulationLog, "==== ModuleMPI: parsing scene settings ============================"<<std::endl;)
        XMLNodeType procTopo = sceneSettings.child("MPISettings").child("ProcessTopology");
        CHECK_XMLNODE(procTopo,"MPISettings/ProcessTopology does not exist");

        std::string type = procTopo.attribute("type").value();

        if(type=="grid") {

            m_topoSettings->m_type = TopologyBuilderSettingsType::TopologyBuilderEnumType::GRIDBUILDER;

            if(!Utilities::stringToVector3(m_topoSettings->m_gridBuilderSettings.m_processDim
                                           ,  procTopo.attribute("dimension").value())) {
                THROWEXCEPTION("---> String conversion in parseMPISettings: dimension failed");
            }

        } else {
            THROWEXCEPTION("---> String conversion in MPISettings:ProcessTopology:type failed: not a valid setting");
        }
        LOGSCLEVEL1(this->m_pSimulationLog, "==================================================================="<<std::endl;)
    }

};

};

/** These module types are defined when there is no derivation from scene parser */
template<typename TSceneParser, typename TDynamicsSystem>
struct SceneParserMPITraits : public SceneParserBaseTraits<TSceneParser,TDynamicsSystem>{

    using SettingsModuleType         = ParserModules::SettingsModuleMPI<SceneParserMPITraits>;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserMPITraits>;
    using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserMPITraits>;
    using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserMPITraits> ;

    using BodyModuleType             = ParserModules::BodyModule< SceneParserMPITraits > ;
    using GeometryModuleType         = ParserModules::GeometryModule<SceneParserMPITraits>;

    using VisModuleType              = ParserModules::VisModuleDummy<SceneParserMPITraits>;

    using MPIModuleType              = ParserModules::MPIModule<SceneParserMPITraits>;

};

template<typename TDynamicsSystem>
class SceneParserMPI: public SceneParser<TDynamicsSystem, SceneParserMPITraits, SceneParserMPI<TDynamicsSystem> > {
private:
    using BaseType = SceneParser<TDynamicsSystem, SceneParserMPITraits, SceneParserMPI<TDynamicsSystem> >;
public:
    using DynamicsSystemType = TDynamicsSystem;

public:
    template<typename ModuleGeneratorType>
    SceneParserMPI(ModuleGeneratorType & moduleGen): BaseType(moduleGen){

    }
};

//
//class SceneParserMPI : public SceneParser {
//public:
//
//    DEFINE_CONFIG_TYPES
//
//    SceneParserMPI(std::shared_ptr<DynamicsSystemType> pDynSys,
//                   std::shared_ptr<MPILayer::ProcessCommunicator > procComm)
//        : SceneParser(pDynSys), m_pProcCommunicator(procComm) {
//        m_nGlobalSimBodies = 0;
//    }
//
//
//    template<typename BodyRangeType>
//    bool parseScene( const boost::filesystem::path & file,
//                     BodyRangeType&& range,
//                     bool parseOnlyVisualizationProperties = false;
//                     bool parseSceneSettings = true,
//                     bool parseProcessTopology = true
//    {
//        m_parseBodies = true;
//        m_useBodyRange = true;
//        m_bodyRange = std::forward<BodyRangeType>(range);
//
//        m_parseOnlyVisualizationProperties = parseOnlyVisualizationProperties;
//        m_parseSceneSettings = parseSceneSettings;
//        m_parseProcessTopoplogy = parseProcessTopology;
//
//        // If the range of bodies is empty, dont parse bodies and dont use the m_bodyRange
//        if(m_bodyRange.empty() ){
//            m_parseBodies = false;
//            m_useBodyRange = false;
//            m_parseOnlyVisualizationProperties = false;
//        }
//        ERROR("Parsing a range of bodies has not been implemented yet")
//        parseSceneIntern(file);
//    }
//
//
//
//
//
//    unsigned int getNumberOfGlobalSimBodies() {
//        return m_nGlobalSimBodies;
//    }
//
//
//protected:
//
//
//    bool parseSceneIntern( boost::filesystem::path file)
//    {
//        using namespace std;
//        m_currentParseFilePath = file;
//        m_currentParseFileDir = m_currentParseFilePath.parent_path();
//
//        m_pSimulationLog->logMessage("---> Parsing Scene...");
//        LOG( m_pSimulationLog,"---> Scene Input file: "  << file.string() <<std::endl; );
//
//
//        //Reset all variables
//        m_nSimBodies = 0;
//        m_nBodies = 0;
//        m_nGlobalSimBodies = 0;
//        m_globalMaxGroupId = 0;
//        m_bodyListGroup.clear();
//        m_bodyScalesGroup.clear();
//        m_initStatesGroup.clear();
//
//        try {
//            m_xmlDoc.LoadFile(m_currentParseFilePath.string());
//
//            m_pSimulationLog->logMessage("---> File successfully loaded ...");
//
//            m_pSimulationLog->logMessage("---> Try to parse the scene ...");
//
//            m_xmlRootNode = m_xmlDoc.FirstChild("DynamicsSystem");
//            if(m_xmlRootNode) {
//                ticpp::Node *node = nullptr;
//
//                node = node = m_xmlRootNode->FirstChild("MPISettings");
//                parseMPISettings(node);
//                m_pSimulationLog->logMessage("---> Parsed MPISettings...");
//
//                node = m_xmlRootNode->FirstChild("SceneSettings");
//                this->parseSceneSettings(node);
//                m_pSimulationLog->logMessage("---> Parsed SceneSettings...");
//
//                node = m_xmlRootNode->FirstChild("SceneObjects");
//                this->parseSceneObjects(node);
//                m_pSimulationLog->logMessage("---> Parsed SceneObjects...");
//
//                node = m_xmlRootNode->FirstChild("SceneSettings");
//                parseSceneSettings2(node);
//                m_pSimulationLog->logMessage("---> Parsed SceneSettings (second part)...");
//
//            } else {
//                m_pSimulationLog->logMessage("---> No DynamicsSystem Node found in XML ...");
//                return false;
//            }
//
//        } catch(ticpp::Exception& ex) {
//            LOG(m_pSimulationLog,"---> Scene XML error: "  << ex.what() << std::endl;);
//            ERRORMSG( "Scene XML error: "  << ex.what() <<std::endl );
//        }
//
//
//        // Filter all bodies according to MPI Grid
//        m_pSimulationLog->logMessage("---> Filter bodies ...");
//        filterBodies();
//
//        //ASSERTMSG(false,"XML parsing...");
//
//        return true;
//    }
//
//    void parseSceneObjects( ticpp::Node *sceneObjects) {
//
//        LOG(m_pSimulationLog,"---> Process SceneObjects ..."<<std::endl;);
//
//        ticpp::Iterator< ticpp::Node > child;
//
//        for ( child = child.begin( sceneObjects ); child != child.end(); child++ ) {
//
//            if( child->Value() == "RigidBodies") {
//                parseRigidBodies( &(*child) );
//            }
//
//        }
//    }
//
//    void parseMPISettings( ticpp::Node *mpiSettings ) {
//
//        ticpp::Element *elem = mpiSettings->FirstChild("ProcessTopology",true)->ToElement();
//        std::string type = elem->GetAttribute("type");
//        if(type=="grid") {
//
//            Vector3 minPoint, maxPoint;
//            if(!Utilities::stringToVector3(minPoint,  elem->GetAttribute("minPoint"))) {
//                throw ticpp::Exception("---> String conversion in parseMPISettings: minPoint failed");
//            }
//            if(!Utilities::stringToVector3(maxPoint,  elem->GetAttribute("maxPoint"))) {
//                throw ticpp::Exception("---> String conversion in parseMPISettings: maxPoint failed");
//            }
//
//            MyMatrix<unsigned int>::Vector3 dim;
//            if(!Utilities::stringToVector3(dim,  elem->GetAttribute("dimension"))) {
//                throw ticpp::Exception("---> String conversion in parseMPISettings: dimension failed");
//            }
//            // saftey check
//            if(dim(0)*dim(1)*dim(2) != m_pProcCommunicator->getNProcesses()) {
//                LOG(m_pSimulationLog,"---> Grid and Process Number do not match!: Grid: ("<< dim.transpose() << ")"<< " with: " << m_pProcCommunicator->getNProcesses() <<" Processes"<<std::endl; );
//                sleep(2);
//                throw ticpp::Exception("---> You have launched to many processes for the grid!");
//            }
//
//            m_pProcCommunicator->createProcTopoGrid(minPoint,maxPoint, dim);
//
//        } else {
//            throw ticpp::Exception("---> String conversion in MPISettings:ProcessTopology:type failed: not a valid setting");
//        }
//
//
//        // Process special Inclusion solver settings
//        elem = mpiSettings->FirstChild("InclusionSolverSettings",true)->ToElement();
//        PREC splitNodeUpdateRatio;
//        if(!Utilities::stringToType(splitNodeUpdateRatio,  elem->GetAttribute("splitNodeUpdateRatio"))) {
//                throw ticpp::Exception("---> String conversion in MPISettings::InclusionSolverSettings: splitNodeUpdateRatio failed");
//        }
//        if(splitNodeUpdateRatio <= 0){
//            throw ticpp::Exception("---> MPISettings::InclusionSolverSettings: splitNodeUpdateRatio <= 0");
//        }
//
//        PREC convergenceCheckRatio;
//        if(!Utilities::stringToType(convergenceCheckRatio,  elem->GetAttribute("convergenceCheckRatio"))) {
//                throw ticpp::Exception("---> String conversion in MPISettings::InclusionSolverSettings: convergenceCheckRatio failed");
//        }
//        if(convergenceCheckRatio <= 0){
//            throw ticpp::Exception("---> MPISettings::InclusionSolverSettings: convergenceCheckRatio <= 0");
//        }
//
//        InclusionSolverSettingsType settIncl;
//        m_pDynSys->getSettings(settIncl);
//        settIncl.m_splitNodeUpdateRatio = splitNodeUpdateRatio;
//        settIncl.m_convergenceCheckRatio = convergenceCheckRatio;
//        m_pDynSys->setSettings(settIncl);
//    }
//
//    void parseRigidBodies( ticpp::Node * rigidbodies ) {
//
//        //Clear current body list;
//        ticpp::Element* rigidBodiesEl = rigidbodies->ToElement();
//        m_bodyListGroup.clear();
//        m_bodyScalesGroup.clear();
//        m_initStatesGroup.clear();
//
//        unsigned int instances = rigidbodies->ToElement()->GetAttribute<unsigned int>("instances");
//
//        unsigned int groupId, startIdx;
//        if(rigidBodiesEl->HasAttribute("groupId")){
//            groupId = rigidBodiesEl->GetAttribute<unsigned int>("groupId");
//            m_globalMaxGroupId = std::max(m_globalMaxGroupId,groupId);
//        }else{
//            m_globalMaxGroupId++;
//            groupId = m_globalMaxGroupId;
//        }
//
//
//        // Get the startidx for this group
//        auto it = this->groupIdToNBodies.find(groupId);
//        if( it == this->groupIdToNBodies.end()){
//            this->groupIdToNBodies[groupId] = startIdx = 0;
//        }else{
//            startIdx = this->groupIdToNBodies[groupId];
//        }
//
//        // update the number of bodies
//        this->groupIdToNBodies[groupId] += instances;
//
//
//        for(int i=0; i<instances; i++) {
//
//
//            RigidBodyType * temp_ptr = new RigidBodyType(RigidBodyId::makeId(groupId,startIdx+i));
//            m_bodyListGroup.push_back(temp_ptr);
//
//            m_bodyScalesGroup.emplace_back(1,1,1);
//        }
//
//
//        ticpp::Node * geometryNode = rigidbodies->FirstChild("Geometry");
//        this->parseGeometry(geometryNode);
//
//
//        ticpp::Node * dynPropNode = rigidbodies->FirstChild("DynamicProperties");
//        this->parseOnlyVisualizationProperties(dynPropNode);
//
//
//        //Copy the pointers!
//
//        if(m_eBodiesState == RigidBodyType::BodyMode::SIMULATED) {
//
//            m_nGlobalSimBodies += m_bodyListGroup.size();
//
//            typename std::vector<RigidBodyType*>::iterator bodyIt;
//            //LOG(m_pSimulationLog, "---> SIZE: " << m_bodyListGroup.size() << std::endl)
//            for(bodyIt= m_bodyListGroup.begin(); bodyIt!=m_bodyListGroup.end();  ) {
//                    if(! m_pDynSys->m_simBodies.addBody((*bodyIt))){
//                        ERRORMSG("Could not add body to m_simBodies! Id: " << RigidBodyId::getBodyIdString(*bodyIt) << " already in map!");
//                    };
//
//                    //LOG(m_pSimulationLog, "---> Added Body with ID: " << RigidBodyId::getBodyIdString(*bodyIt)<< std::endl);
//
//
//                    m_nSimBodies++;
//                    m_nBodies++;
//
//                    ++bodyIt;
//            }
//
//            // Copy all init states
//            LOG(m_pSimulationLog, "---> Copy init states... " << std::endl;);
////            for(auto it = m_initStatesGroup.begin(); it!=m_initStatesGroup.end();it++){
////                LOG(m_pSimulationLog, "\t---> state id: " << RigidBodyId::getBodyIdString(it->first)
////                    << std::endl << "\t\t---> q: " << it->second.m_q.transpose()
////                    << std::endl << "\t\t---> u: " << it->second.m_u.transpose() << std::endl;
////                    );
////            }
//            m_pDynSys->m_bodiesInitStates.insert( m_initStatesGroup.begin(), m_initStatesGroup.end() );
//
//
//
//        } else if(m_eBodiesState == RigidBodyType::BodyMode::STATIC) {
//
//           typename std::vector<RigidBodyType*>::iterator bodyIt;
//
//            for(bodyIt= m_bodyListGroup.begin(); bodyIt!=m_bodyListGroup.end(); bodyIt++) {
//
//                if(! m_pDynSys->m_staticBodies.addBody((*bodyIt))){
//                        ERRORMSG("Could not add body to m_staticBodies! Id: " << RigidBodyId::getBodyIdString(*bodyIt) << " already in map!");
//                };
//
//                m_nBodies++;
//            }
//        } else {
//            throw ticpp::Exception("---> Adding only simulated and not simulated objects supported!");
//        }
//
//
////        ticpp::Node * visualizationNode = rigidbodies->FirstChild("Visualization");
////        this->parseVisualization( visualizationNode);
//
//
//        //Remove all bodies from the sceneparsers intern list!
//        m_bodyListGroup.clear();
//        m_bodyScalesGroup.clear();
//        m_initStatesGroup.clear();
//    }
//
//    void filterBodies(){
//
//        auto & simBodies = m_pDynSys->m_simBodies;
//        LOG(m_pSimulationLog, "---> Reject Body with ID: ")
//        for(auto bodyIt= simBodies.begin(); bodyIt!=simBodies.end();
//        /*No incremente because we delete inside the loop invalidating iterators*/ ) {
//        // Check if Body belongs to the topology! // Check CoG!
//                if(!m_pProcCommunicator->getProcTopo()->belongsPointToProcess((*bodyIt)->m_r_S)) {
//                    LOG(m_pSimulationLog, RigidBodyId::getBodyIdString(*bodyIt));
//
//                    // Delete the init state
//                    m_pDynSys->m_bodiesInitStates.erase((*bodyIt)->m_id);
//                    // Delete this body immediately!
//                    bodyIt = simBodies.deleteBody(bodyIt);
//
//                    m_nSimBodies--;
//                }else{
//                    bodyIt++;
//                }
//        }
//        LOG(m_pSimulationLog, std::endl);
//    }
//
//    std::shared_ptr< MPILayer::ProcessCommunicator > m_pProcCommunicator;
//
//    unsigned int m_nGlobalSimBodies;
//
//    //using declerations
//    using SceneParser::m_globalMaxGroupId;
//    using SceneParser::m_nSimBodies;
//    using SceneParser::m_nBodies;
//    using SceneParser::m_parseOnlyVisualizationProperties;
//    using SceneParser::m_pDynSys;
//    using SceneParser::m_currentParseFilePath;
//    using SceneParser::m_currentParseFileDir;
//    using SceneParser::m_xmlDoc;
//    using SceneParser::m_xmlRootNode;
//    using SceneParser::m_pSimulationLog;
//    using SceneParser::logstream;
//    // Temprary structures
//    using SceneParser::m_eBodiesState;
//    using SceneParser::m_bodyListGroup;
//    using SceneParser::m_bodyScalesGroup;
//    using SceneParser::m_initStatesGroup;
//
//};


#endif

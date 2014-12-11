#ifndef GRSF_Systems_SceneParser_hpp
#define GRSF_Systems_SceneParser_hpp

#include <memory>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Common/AssertionDebug.hpp"


#include "GRSF/Common/XMLMacros.hpp"
#include "GRSF/Systems/SceneParserModules.hpp"


/** The traits for a standart SceneParser class*/
template<typename TSceneParser, typename TDynamicsSystem>
struct SceneParserTraits: SceneParserBaseTraits<TSceneParser,TDynamicsSystem> {
    // Module typedefs
    using SettingsModuleType         = ParserModules::SettingsModule<SceneParserTraits>;
    using ExternalForcesModuleType   = ParserModules::ExternalForcesModule<SceneParserTraits>;
    using ContactParamModuleType     = ParserModules::ContactParamModule<SceneParserTraits>;
    using InitStatesModuleType       = ParserModules::InitStatesModule<SceneParserTraits> ;

    using BodyMStaticOptions         = ParserModules::BodyModuleStaticOptions<>;
    using BodyModuleType             = ParserModules::BodyModule< SceneParserTraits > ;

    using GeomMStaticOptions         = ParserModules::GeometryModuleStaticOptions<>;
    using GeometryModuleType         = ParserModules::GeometryModule<SceneParserTraits, GeomMStaticOptions>;

    using VisModuleType              = ParserModules::VisModuleDummy<SceneParserTraits>;

    using MPIModuleType              = ParserModules::MPIModuleDummy<SceneParserTraits>;
};


/** SceneParser Options */
struct SceneParserDynamicOptions {
    bool m_parseSceneSettings = true;  ///< Parse SceneSettings (default= true)
    bool m_parseSceneObjects  = true;  ///< Parse SceneObjects, (default= true)
};



template<
 typename TDynamicsSystem,
 template<typename P, typename D> class TParserTraits = SceneParserTraits,
 typename TDerived = void >
class SceneParser {
public:

    using DerivedType = typename std::conditional< std::is_same<TDerived,void>::value, SceneParser, TDerived>::type;

    /** Modules defintions
    * This type traits define the module types from TParserTraits
    * SceneParser is injected into the modules, we use this class instead of the derived one
    */
    using ParserForModulesType = SceneParser;
    using ParserTraits = TParserTraits<ParserForModulesType, TDynamicsSystem >;
    DEFINE_PARSER_TYPE_TRAITS(ParserTraits);

    using SceneParserDynamicOptionsType = SceneParserDynamicOptions;
    using BodyModuleDynamicOptionsType  = typename BodyModuleType::DynamicOptionsType;
public:

    /**
    * Constructor takes a module function which constructs all modules.
    * If a xmlDoc pointer is given, this document is taken
    */
    template<typename ModuleGeneratorType>
    SceneParser(ModuleGeneratorType & moduleGen, Logging::Log * log) {

        m_pSimulationLog = log;
        ASSERTMSG(m_pSimulationLog, "Log pointer is zero!");

        // Get all Modules from the Generator
        std::tie(m_pSettingsModule,
                 m_pExternalForcesModule,
                 m_pContactParamModule,
                 m_pInitStatesModule,
                 m_pBodyModule,
                 m_pGeometryModule,
                 m_pVisModule,
                 m_pMPIModule)
            = moduleGen.template createParserModules<ParserForModulesType>( static_cast<ParserForModulesType*>(this));

    }

    /**
    * Set a XML Document by specifying a pointer and a filepath if a document is already been loaded!
    */
    void setDocument(std::shared_ptr<pugi::xml_document> xmlDoc, boost::filesystem::path file = "" ){
        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFileDir = m_currentParseFilePath.parent_path();
        if(xmlDoc){
            ERRORMSG("XML Document pointer is null!")
        }
        m_xmlDoc = xmlDoc;
    }

    /**
    * Load a file, this does not parse the file, it only loads the DOM tree!
    */
    void loadFile(const boost::filesystem::path & file){
        if(!boost::filesystem::exists(file)) {
            ERRORMSG("Scene Input file does not exist!");
        }
        pugi::xml_parse_result result = m_xmlDoc->load_file(file.c_str());
        if (result) {
            LOGSCLEVEL1(m_pSimulationLog, "---> Loaded XML [" << file.string() << "] without errors!" << std::endl;);
        } else {
            ERRORMSG( "Loaded XML [" << file.string() << "] with errors!" << std::endl
                            << "Error description: " << result.description() << std::endl
                            << "Error offset: " << result.offset )
        }

        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();
    }

    /**
    * range is only applied to the groups with the attribute enableSelectiveIds="true"
    */
    template<typename TParserTraitsOptions = SceneParserDynamicOptions,
             typename TBodyParserOptions = BodyModuleDynamicOptionsType>
    bool parseScene( const boost::filesystem::path & file,
                     TParserTraitsOptions&& opt = TParserTraitsOptions(),
                     TBodyParserOptions&& optBody = TBodyParserOptions()
                   ) {

        // Forward all settings
        m_opts = std::forward<TParserTraitsOptions>(opt);

        if(m_pBodyModule) {
            m_pBodyModule->setParsingOptions(std::forward<TBodyParserOptions>(optBody));
        }

        if(m_pGeometryModule) {
            //m_pGeometryModule->setParsingOptions(std::forward<TGeometryModuleOptions>(optGeom));
        }

        parseSceneIntern(file);

        return true;
    }


    virtual void cleanUp() {
        // Delegate all cleanUp stuff to the modules!
        if(m_pSettingsModule) {
            m_pSettingsModule->cleanUp();
        }
        if(m_pContactParamModule) {
            m_pContactParamModule->cleanUp();
        }
        if(m_pExternalForcesModule) {
            m_pExternalForcesModule->cleanUp();
        }
        if(m_pGeometryModule) {
            m_pGeometryModule->cleanUp();
        }
        if(m_pBodyModule) {
            m_pBodyModule->cleanUp();
        }
        if(m_pInitStatesModule) {
            m_pInitStatesModule->cleanUp();
        }
        if(m_pVisModule) {
            m_pVisModule->cleanUp();
        }
    }


    boost::filesystem::path getParsedSceneFile() {
        return m_currentParseFilePath;
    }

    void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            ERRORMSG("---> The file ' " + file.string() + "' does not exist!");
        }
    }

    LogType * getSimLog() {
        return m_pSimulationLog;
    }

    unsigned int getSpecifiedSimBodies() {
        if(m_pBodyModule) {
            return m_pBodyModule->getSpecifiedSimBodies();
        }
        return 0;
    }

    GeometryModuleType * getGeometryModule() {
        return m_pGeometryModule;
    }
    SettingsModuleType * getSettingsModule() {
        return m_pSettingsModule;
    }
    ExternalForcesModuleType * getExternalForcesModule() {
        return m_pExternalForcesModule;
    }
    ContactParamModuleType * getContactParamModule() {
        return m_pContactParamModule;
    }
    BodyModuleType * getBodyModule() {
        return m_pBodyModule;
    }
    InitStatesModuleType * getInitStatesModule() {
        return m_pInitStatesModule;
    }
    VisModuleType * getVisModule() {
        return m_pVisModule;
    }
    MPIModuleType * getMPIModule() {
        return m_pMPIModule;
    }

protected:

    bool parseSceneIntern(const boost::filesystem::path & file) {

        LOGSCLEVEL1( m_pSimulationLog, "---> SceneParser parsing: ========================================================" <<
                     std::endl << "\t file: " << file <<std::endl;);
        LOGSCLEVEL1( m_pSimulationLog, "---> SceneParser Options: " <<std::endl <<
                     "\t parse scene settings:"<<m_opts.m_parseSceneSettings << std::endl<<
                     "\t parse scene objects:"<<m_opts.m_parseSceneObjects << std::endl;);

         // if document does not exist create one and force loading!
        if(!m_xmlDoc){
            m_xmlDoc = std::shared_ptr<pugi::xml_document>( new pugi::xml_document() );
        }
        if( file.empty() ){
            ERRORMSG("File name is empty!");
        }



        // Load the file if necessary
        if(file != m_currentParseFilePath) {
            loadFile(file);
        }

        LOGSCLEVEL1( m_pSimulationLog, "---> Scene Input file: "  << file.string() <<std::endl; );

        LOGSCLEVEL1(m_pSimulationLog, "---> Try to parse the scene ..."<<std::endl;);

        GET_XMLCHILDNODE_CHECK( m_xmlRootNode, "DynamicsSystem" , (*m_xmlDoc) );

        XMLNodeType node;
        GET_XMLCHILDNODE_CHECK( node , "SceneSettings",  m_xmlRootNode);
        parseSceneSettingsPre(node);

        node = m_xmlRootNode.child("SceneObjects");
        parseSceneObjects(node);

        node = m_xmlRootNode.child("SceneSettings");
        parseSceneSettingsPost(node);

        //parseOtherOptions(m_xmlRootNode);

        LOGSCLEVEL1( m_pSimulationLog, "---> SceneParser finshed =========================================================" << std::endl;);

        return true;
    }


    virtual void parseSceneSettingsPre( XMLNodeType sceneSettings ) {

        if(!m_opts.m_parseSceneSettings) {
            LOGSCLEVEL1(m_pSimulationLog,"---> Skip SceneSettings"<<std::endl;);
            return;
        }

        LOGSCLEVEL1(m_pSimulationLog,"---> Parse Pre SceneSettings..."<<std::endl;);

        if(m_pSettingsModule) {
            m_pSettingsModule->parse(sceneSettings);
        }

        if(m_pContactParamModule) {
            m_pContactParamModule->parse(sceneSettings);
        }

        if(m_pExternalForcesModule) {
            m_pExternalForcesModule->parse(sceneSettings);
        }

        if(m_pGeometryModule) {
            m_pGeometryModule->parseGlobalGeometries(sceneSettings);
        }

        if(m_pBodyModule) {
            m_pBodyModule->parseModuleOptions(sceneSettings);
        }
    }

    virtual void parseSceneSettingsPost( XMLNodeType sceneSettings ) {

        if(!m_opts.m_parseSceneSettings) {
            LOGSCLEVEL1(m_pSimulationLog,"---> Skip SceneSettings"<<std::endl;);
            return;
        }

        LOGSCLEVEL1(m_pSimulationLog,"---> Parse Post SceneSettings..."<<std::endl;);

        if(m_pInitStatesModule) {
            m_pInitStatesModule->parseGlobalInitialCondition(sceneSettings);
        }

        if(m_pVisModule) {
            m_pVisModule->parseSceneSettingsPost(sceneSettings);
        }

        if(m_pMPIModule) {
            m_pMPIModule->parseSceneSettingsPost(sceneSettings);
        }
    }

    virtual void parseSceneObjects( XMLNodeType sceneObjects) {
        if(m_pBodyModule) {
            m_pBodyModule->parse(sceneObjects);
        }
    }

    std::shared_ptr<DynamicsSystemType> m_pDynSys;

    SceneParserDynamicOptionsType m_opts;

    boost::filesystem::path m_currentParseFilePath;
    boost::filesystem::path m_currentParseFileDir;

    /** XML Declarations */
    std::shared_ptr<pugi::xml_document> m_xmlDoc;
    bool m_loadedFile = false;
    pugi::xml_node m_xmlRootNode;

    /** Log */
    LogType * m_pSimulationLog;

    /** Modules */
    std::unique_ptr< SettingsModuleType>       m_pSettingsModule;
    std::unique_ptr< GeometryModuleType>       m_pGeometryModule;
    std::unique_ptr< ExternalForcesModuleType> m_pExternalForcesModule;
    std::unique_ptr< ContactParamModuleType>   m_pContactParamModule;

    std::unique_ptr< BodyModuleType>           m_pBodyModule;
    std::unique_ptr< InitStatesModuleType>     m_pInitStatesModule;

    std::unique_ptr< VisModuleType>            m_pVisModule;

    std::unique_ptr< MPIModuleType>            m_pMPIModule;
};



#endif


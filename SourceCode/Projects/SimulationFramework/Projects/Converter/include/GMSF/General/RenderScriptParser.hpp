#ifndef GMSF_General_RenderScriptParser_hpp
#define GMSF_General_RenderScriptParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include "pugixml.hpp"

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "AssertionDebug.hpp"

#include "XMLMacros.hpp"
#include "RenderScriptParserBaseTraits.hpp"
#include "RenderScriptParserModules.hpp"

/** The traits for a standart RenderScriptParser class*/
template<typename TSceneParser, typename TCollection>
struct RenderScriptParserTraits : RenderMatParserBaseTraits<TSceneParser,TCollection> {
    // Module typedefs
    using MaterialsModuleType   = typename RenderScriptParserModules::MaterialsModule<RenderScriptParserTraits>;
    using MatGenModuleType   = typename RenderScriptParserModules::ScriptGeneratorModule<RenderScriptParserTraits>;
};

template< typename TCollection, template<typename P, typename C> class TParserTraits = RenderScriptParserTraits >
class RenderScriptParser {
public:

    using ParserForModulesType = RenderScriptParser;
    using ParserTraits = TParserTraits<ParserForModulesType, TCollection>;
    DEFINE_MATCOLPARSER_TYPE_TRAITS(ParserTraits);
private:

    boost::filesystem::path m_currentParseFilePath;
    boost::filesystem::path m_currentParseFileDir;

    /** XML Declarations */
    std::shared_ptr<pugi::xml_document> m_xmlDoc;
    pugi::xml_node m_xmlRootNode;

    /** Log */
    LogType * m_pLog;

    /** Modules */
    std::unique_ptr< MaterialsModuleType >       m_pMaterialsModule;
    std::unique_ptr< MatGenModuleType >          m_pMaterialGeneratorModule;

public:

    /**
    * Constructor takes a module function which constructs all modules.
    * If a xmlDoc pointer is given, this document is taken
    */
    template<typename ModuleGeneratorType>
    RenderScriptParser(ModuleGeneratorType & moduleGen, Logging::Log * log) {
        m_pLog = log;
        ASSERTMSG(m_pLog, "Log pointer is zero!");
        // Get all Modules from the Generator
        std::tie(m_pMaterialsModule, m_pMaterialGeneratorModule) = moduleGen.template createParserModules<ParserForModulesType>( static_cast<ParserForModulesType*>(this));
    }


    void parse(const boost::filesystem::path & file) {
        parseSceneIntern(file);
    }

    void cleanUp() {
        // Delegate all cleanUp stuff to the modules!
        if(m_pMaterialsModule) {
            m_pMaterialsModule->cleanUp();
        }
    }

    boost::filesystem::path getParsedSceneFile() {
        return m_currentParseFilePath;
    }

    LogType * getLog(){return m_pLog;}

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
    void loadFile(const boost::filesystem::path & file) {
        if(!boost::filesystem::exists(file)) {
            ERRORMSG("Scene Input file does not exist!");
        }
        pugi::xml_parse_result result = m_xmlDoc->load_file(file.c_str());
        if (result) {
            LOGMCLEVEL1(m_pLog, "---> Loaded XML [" << file.string() << "] without errors!" << std::endl;);
        } else {
            ERRORMSG( "Loaded XML [" << file.string() << "] with errors!" << std::endl
                            << "Error description: " << result.description() << std::endl
                            << "Error offset: " << result.offset )
        }

        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();
    }

private:

    void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            ERRORMSG("---> The file ' " + file.string() + "' does not exist!");
        }
    }

    void parseSceneIntern(const boost::filesystem::path & file) {

        LOGMCLEVEL1( m_pLog, "---> RenderScriptParser parsing: ========================================================" <<
                     std::endl << "\t file: " << file <<std::endl;);

        LOGMCLEVEL1( m_pLog, "---> Input file: "  << file.string() <<std::endl; );


        if(!boost::filesystem::exists(file)) {
            ERRORMSG("Scene Input file does not exist!");
        }

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

        try {

            LOGMCLEVEL1(m_pLog, "---> Try to parse the file ..."<<std::endl;);

            GET_XMLCHILDNODE_CHECK( m_xmlRootNode, "Render" , (*m_xmlDoc) );


            XMLNodeType node = m_xmlRootNode.child("Materials");
            if(node && m_pMaterialsModule) {
                m_pMaterialsModule->parse(node);
            }

            node = m_xmlRootNode.child("ScriptGenerator");
            if(node && m_pMaterialGeneratorModule) {
                m_pMaterialGeneratorModule->parse(node, m_pMaterialsModule->getMaterialMap() );
            }


        } catch(Exception& ex) {
            LOGMCLEVEL1(m_pLog,  "Scene XML error: "  << ex.what() <<std::endl);
            ERRORMSG( "Scene XML error: "  << ex.what() );
        }

        LOGMCLEVEL1( m_pLog, "---> RenderScriptParser finshed =========================================================" << std::endl;);

    }

};


#endif


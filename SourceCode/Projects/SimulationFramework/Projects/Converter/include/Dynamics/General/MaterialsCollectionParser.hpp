#ifndef MaterialsCollectionParser_hpp
#define MaterialsCollectionParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include "pugixml.hpp"

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"
#include "LogDefines.hpp"
#include "Exception.hpp"

#include "XMLMacros.hpp"

#define DEFINE_MATCOLLPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using ParserType = typename TParserTraits::ParserType; \
    using CollectionType = typename TParserTraits::CollectionType; \
    using LogType = typename TParserTraits::LogType; \
    using XMLNodeType = typename TParserTraits::XMLNodeType;\
    using XMLNodeItType = typename TParserTraits::XMLNodeItType;\
    using XMLAttributeType = typename TParserTraits::XMLAttributeType;\
    using RandomGenType = typename TParserTraits::RandomGenType; \
    template<typename T> using UniformDistType = typename TParserTraits::template UniformDistType<T>;

#define  DEFINE_MATCOLPARSER_TYPE_TRAITS( TParserTraits )  \
    DEFINE_MATCOLLPARSER_BASE_TYPE_TRAITS( TParserTraits ) \
    using MaterialsModuleType     = typename TParserTraits::MaterialsModuleType;\


namespace MatCollParserModules {

    template<typename TParserTraits>
    class MaterialsModule {
    public:
        DEFINE_MATCOLPARSER_TYPE_TRAITS(TParserTraits)

        using MaterialsMapType = typename CollectionType::MaterialsMapType;

        MaterialsModule(ParserType * p, MaterialsMapType * m):m_parser(p),m_materials(m), m_pLog(p->getLog()) {}

        void parse(XMLNodeType & materialNode) {
            auto nodes = materialNode.children("Material");
            auto itNodeEnd = nodes.end();
            for (auto itNode = nodes.begin(); itNode != itNodeEnd; ++itNode) {
                 unsigned int id;
                 if(!Utilities::stringToType(id, itNode->attribute("id").value())) {
                      THROWEXCEPTION("---> String conversion in Material: id failed");
                 }
                 ASSERTMSG(itNode->value()," String in material id: " << id << "is empty!")
                 m_materials->emplace(id, itNode->value() );
                 LOGMCLEVEL3(m_pLog,"---> Parsed Material with id: " << id << std::endl;)
            }
        }

        void cleanUp() {
        }

    private:
        ParserType * m_parser;
        LogType * m_pLog;
        MaterialsMapType * m_materials;
    };
};



/** The base traits for every MaterialsCollectionParser parser */
template<typename TSceneParser, typename TCollection>
struct MatCollParserBaseTraits {


    using CollectionType = TCollection;

    using ParserType = TSceneParser;
    using LogType = Logging::Log;

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using RandomGenType = typename CollectionType::RandomGenType;
    template<typename T>
    using UniformDistType = std::uniform_real_distribution<T>;
};

/** The traits for a standart MatCollParser class*/
template<typename TSceneParser, typename TCollection>
struct MatCollParserTraits : MatCollParserBaseTraits<TSceneParser,TCollection> {
    // Module typedefs
    using MaterialsModuleType   = typename MatCollParserModules::MaterialsModule<MatCollParserTraits>;
};

template< typename TCollection, template<typename P, typename C> class TParserTraits = MatCollParserTraits >
class MaterialsCollectionParser {
public:

    using ParserForModulesType = MaterialsCollectionParser;
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

public:

    /**
    * Constructor takes a module function which constructs all modules.
    * If a xmlDoc pointer is given, this document is taken
    */
    template<typename ModuleGeneratorType>
    MaterialsCollectionParser(ModuleGeneratorType & moduleGen, Logging::Log * log) {
        m_pLog = log;
        ASSERTMSG(m_pLog, "Log pointer is zero!");
        // Get all Modules from the Generator
        std::tie(m_pMaterialsModule) = moduleGen.template createParserModules<ParserForModulesType>( static_cast<ParserForModulesType*>(this));
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
            THROWEXCEPTION("XML Document pointer is null!")
        }
        m_xmlDoc = xmlDoc;
    }

    /**
    * Load a file, this does not parse the file, it only loads the DOM tree!
    */
    void loadFile(const boost::filesystem::path & file) {
        if(!boost::filesystem::exists(file)) {
            THROWEXCEPTION("Scene Input file does not exist!");
        }
        pugi::xml_parse_result result = m_xmlDoc->load_file(file.c_str());
        if (result) {
            LOGMCLEVEL1(m_pLog, "---> Loaded XML [" << file.string() << "] without errors!" << std::endl;);
        } else {
            THROWEXCEPTION( "Loaded XML [" << file.string() << "] with errors!" << std::endl
                            << "Error description: " << result.description() << std::endl
                            << "Error offset: " << result.offset )
        }

        m_currentParseFilePath = file;
        m_currentParseFileDir = m_currentParseFilePath.parent_path();
    }

private:

    void checkFileExists(boost::filesystem::path file) {
        if( !boost::filesystem::exists(file) ) {
            THROWEXCEPTION("---> The file ' " + file.string() + "' does not exist!");
        }
    }

    void parseSceneIntern(const boost::filesystem::path & file) {

        LOGMCLEVEL1( m_pLog, "---> MaterialsCollectionParser parsing: ========================================================" <<
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
            THROWEXCEPTION("File name is empty!");
        }

        // Load the file if necessary
        if(file != m_currentParseFilePath) {
            loadFile(file);
        }

        try {

            LOGMCLEVEL1(m_pLog, "---> Try to parse the file ..."<<std::endl;);

            GET_XMLCHILDNODE_CHECK( m_xmlRootNode, "MaterialsCollection" , (*m_xmlDoc) );


            XMLNodeType node = m_xmlRootNode.child("Materials");
            if(node && m_pMaterialsModule) {
                m_pMaterialsModule->parse(node);
            }


        } catch(Exception& ex) {
            LOGMCLEVEL1(m_pLog,  "Scene XML error: "  << ex.what() <<std::endl);
            ERRORMSG( "Scene XML error: "  << ex.what() );
        }

        LOGMCLEVEL1( m_pLog, "---> MaterialsCollectionParser finshed =========================================================" << std::endl;);

    }

};


#endif


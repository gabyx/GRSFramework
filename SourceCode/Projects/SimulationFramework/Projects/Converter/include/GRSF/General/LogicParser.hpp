#ifndef GRSF_General_LogicParser_hpp
#define GRSF_General_LogicParser_hpp

#include <vector>
#include <fstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/DemangleTypes.hpp"

#include "GRSF/Common/TupleHelper.hpp"

#include "GRSF/Common/XMLMacros.hpp"

#include "GRSF/General/LogicParserTraits.hpp"

// TODO
// This logicparser is general already, it takes a couple of models and makes them parsing from the root node,
// Make this parser a general base of all parsers (scene parser!)

template< typename TDataStorage,
          template<typename P, typename C> class TParserTraits = LogicParserTraits,
          typename TDerived = void>
class LogicParser {
public:

    using ParserTraits = TParserTraits<LogicParser, TDataStorage>;
    DEFINE_LOGICPARSER_TYPE_TRAITS(ParserTraits);
private:

    boost::filesystem::path m_currentParseFilePath;
    boost::filesystem::path m_currentParseFileDir;

    /** XML Declarations */
    std::shared_ptr<pugi::xml_document> m_xmlDoc;
    pugi::xml_node m_xmlRootNode;

    /** Log */
    LogType * m_pLog;

    /** Modules */
    typename ParserTraits::TupleModules m_modules;

    struct VisitorParse{
        VisitorParse(XMLNodeType & node): n(node) {}
        template<typename M>
        void operator()(M & module){
            if(module){
                module->parse(n);
            }else{
                WARNINGMSG(false, "No module:"  <<  demangle::type(module) << " (pointer = nullptr)" << std::endl)
            }
        }
        XMLNodeType & n;
    };
    struct VisitorCleanUp{
        template<typename M>
        void operator()(M & module){
            if(module){
                module->cleanUp();
            }else{
                WARNINGMSG(false,"No module:"  <<  demangle::type(module) << " (pointer = nullptr)  " << std::endl)
            }
        }
    };

public:

    /**
    * Constructor takes a module function which constructs all modules.
    */
    template<typename ModuleGeneratorType>
    LogicParser(ModuleGeneratorType & moduleGen, Logging::Log * log) {
        m_pLog = log;
        ASSERTMSG(m_pLog, "Log pointer is zero!");
        // Get all Modules from the Generator
        m_modules = moduleGen.template createParserModules<ParserType>( this );
    }


    void parse(const boost::filesystem::path & file) {
        parseSceneIntern(file);
    }

    void cleanUp() {
        VisitorCleanUp up;
        TupleVisit::visit(up,m_modules);
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
            LOGLPLEVEL1(m_pLog, "---> Loaded XML [" << file.string() << "] without errors!" << std::endl;);
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

        LOGLPLEVEL1( m_pLog, "---> LogicParser parsing: ========================================================" <<
                     std::endl << "\t file: " << file <<std::endl;);

        LOGLPLEVEL1( m_pLog, "---> Input file: "  << file.string() <<std::endl; );


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
        if(file != m_currentParseFilePath){
            loadFile(file);
        }

        try {

            LOGLPLEVEL1(m_pLog, "---> Try to parse the file ..."<<std::endl;);

            m_xmlRootNode = m_xmlDoc->first_child();
            CHECK_XMLNODE(m_xmlRootNode,"root: first child")

            VisitorParse v(m_xmlRootNode);
            TupleVisit::visit(v,m_modules);


        } catch(Exception& ex) {
            LOGLPLEVEL1(m_pLog,  "Scene XML error: "  << ex.what() <<std::endl);
            ERRORMSG( "Scene XML error: "  << ex.what() );
        }

        LOGLPLEVEL1( m_pLog, "---> LogicParser finshed =========================================================" << std::endl;);

    }

};


#endif



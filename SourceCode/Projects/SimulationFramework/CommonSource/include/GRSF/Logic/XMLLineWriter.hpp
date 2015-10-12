#ifndef GRSF_Logic_XMLLineWriter_hpp
#define GRSF_Logic_XMLLineWriter_hpp

#include <sstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/Common/XMLMacros.hpp"

#include "GRSF/Logic/LogicNode.hpp"


namespace LogicNodes{

    template<typename TValue>
    class XMLLineWriter : public LogicNode  {
    public:

        using XMLNodeType = pugi::xml_node;
        using XMLDocumentType = pugi::xml_document;
        static const auto nodePCData = pugi::node_pcdata;

        struct Inputs {
            enum {
                File,
                Value,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };

        DECLARE_ISOCKET_TYPE(File, boost::filesystem::path );
        DECLARE_ISOCKET_TYPE(Value, TValue );


        XMLLineWriter(unsigned int id,
                      const boost::filesystem::path filePath = "",
                      std::string rootName="Root", std::string childName="Child"):
            LogicNode(id), m_rootName(rootName),m_childName(childName)
        {
            ADD_ISOCK(File,filePath);
            ADD_ISOCK(Value,TValue());
        }

        void compute() {
            if( GET_ISOCKET_VALUE(File) != m_openedFile ){
                m_openedFile = GET_ISOCKET_VALUE(File);
                openFile(m_openedFile);
            }
            m_stream.str("");
            m_stream << GET_ISOCKET_VALUE(Value);
            m_root.append_child(m_childName.c_str()).append_child(nodePCData).set_value(m_stream.str().c_str());
        }

        void openFile(const boost::filesystem::path & f){
            // file path has changed, close file and open new one
            if(!m_openedFile.empty()){
                m_xmlFile.save_file(m_openedFile.string().c_str());
            }
            m_xmlFile.reset();

            m_root = m_xmlFile.append_child(m_rootName.c_str());
        }

        void finalize(){
            if(!m_openedFile.empty()){
                m_xmlFile.save_file(m_openedFile.string().c_str());
            }
            m_openedFile = "";
        }

        ~XMLLineWriter(){
            if(!m_openedFile.empty()){
                m_xmlFile.save_file(m_openedFile.string().c_str());
            }
            m_openedFile = "";
        }
    private:

        std::stringstream m_stream;

        boost::filesystem::path m_openedFile;
        XMLDocumentType m_xmlFile;
        XMLNodeType m_root;

        std::string m_rootName,m_childName;
    };

};

#endif


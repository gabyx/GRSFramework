// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_logic_XMLLineWriter_hpp
#define GRSF_logic_XMLLineWriter_hpp

#include <sstream>

#include <boost/filesystem.hpp>
#include <pugixml.hpp>

#include "GRSF/common/XMLMacros.hpp"

#include "GRSF/logic/LogicNode.hpp"

namespace LogicNodes
{
template <typename TValue>
class XMLLineWriter : public LogicNode
{
public:
    using XMLNodeType            = pugi::xml_node;
    using XMLDocumentType        = pugi::xml_document;
    static const auto nodePCData = pugi::node_pcdata;

    struct Inputs
    {
        enum
        {
            File,
            Value,
            INPUTS_LAST
        };
    };

    struct Outputs
    {
        enum
        {
            OUTPUTS_LAST
        };
    };

    enum
    {
        N_INPUTS  = Inputs::INPUTS_LAST,
        N_OUTPUTS = Outputs::OUTPUTS_LAST,
        N_SOCKETS = N_INPUTS + N_OUTPUTS
    };

    DECLARE_ISOCKET_TYPE(File, boost::filesystem::path);
    DECLARE_ISOCKET_TYPE(Value, TValue);

    XMLLineWriter(unsigned int                  id,
                  const boost::filesystem::path filePath  = "",
                  std::string                   rootName  = "Root",
                  std::string                   childName = "Child")
        : LogicNode(id), m_rootName(rootName), m_childName(childName)
    {
        ADD_ISOCK(File, filePath);
        ADD_ISOCK(Value, TValue());
    }

    void compute()
    {
        checkFileChange(GET_ISOCKET_VALUE(File));

        m_stream.str("");
        m_stream << GET_ISOCKET_VALUE(Value);
        std::cout << "XMLWriter: " << m_stream.str() << std::endl;
        m_root.append_child(m_childName.c_str()).append_child(nodePCData).set_value(m_stream.str().c_str());
    }

    void checkFileChange(const boost::filesystem::path& f)
    {
        // file path has changed, close file and open new one

        if (f != m_openedFile)
        {
            if (!m_openedFile.empty())
            {
                m_xmlFile.save_file(m_openedFile.string().c_str());
            }
            m_xmlFile.reset();
            m_root       = m_xmlFile.append_child(m_rootName.c_str());
            m_openedFile = f;
        }
    }

    ~XMLLineWriter()
    {
        if (!m_openedFile.empty())
        {
            m_xmlFile.save_file(m_openedFile.string().c_str());
        }
        m_openedFile = "";
    }

private:
    std::stringstream m_stream;

    boost::filesystem::path m_openedFile;
    XMLDocumentType         m_xmlFile;
    XMLNodeType             m_root;

    std::string m_rootName, m_childName;
};
};

#endif

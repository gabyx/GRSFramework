#ifndef GRSF_Converters_AnalyzerConverter_hpp
#define GRSF_Converters_AnalyzerConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Converters/SimFileConverter.hpp"
#include "GRSF/General/SimFileExecutionGraph.hpp"

class AnalyzerConverter : public SimFileConverter {
public:

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using Base = SimFileConverter;
    using ExecutionGraphType = SimFileExecutionGraph;

    AnalyzerConverter(const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path logicFile);

    void convert();


private:



    ExecutionGraphType m_executionGraph;

    void setupExecutionGraph();

    boost::filesystem::path m_logicFile;
    boost::filesystem::path m_sceneFile;

};

#endif // GRSF_Converters_AnalyzerConverter_hpp



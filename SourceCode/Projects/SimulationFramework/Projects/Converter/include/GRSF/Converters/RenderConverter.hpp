#ifndef GRSF_Converters_RenderConverter_hpp
#define GRSF_Converters_RenderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Converters/LogicConverter.hpp"

#include "GRSF/Common/ApplicationCLOptionsConverter.hpp"

#include "GRSF/General/RenderData.hpp"
#include "GRSF/General/RenderExecutionGraph.hpp"

class RenderConverter : public LogicConverter<RenderExecutionGraph> {
public:

    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using Base = LogicConverter<RenderExecutionGraph>;
    using ExecutionGraphType = typename LogicConverter::ExecutionGraphType;

    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path outputDir,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path logicFile,
                  Renderer renderer);


private:

    RenderData m_renderData;
    Renderer m_renderer;

    virtual void setupExecutionGraph();



};

#endif // GRSF_Converters_RenderConverter_hpp


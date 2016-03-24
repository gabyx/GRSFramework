#ifndef GRSF_converters_renderer_RenderConverter_hpp
#define GRSF_converters_renderer_RenderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/converters/SimFileConverter.hpp"

#include "GRSF/common/ApplicationCLOptionsConverter.hpp"

#include "GRSF/converters/renderer/RenderData.hpp"
#include "GRSF/converters/renderer/RenderExecutionGraph.hpp"

class RenderConverter : public SimFileConverter {
public:

    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using Base = SimFileConverter;
    using ExecutionGraphType = RenderExecutionGraph;

    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    RenderConverter(const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path logicFile,
                  Renderer renderer);
    void convert( );


private:

    RenderExecutionGraph m_executionGraph;
    RenderData m_renderData;
    Renderer m_renderer;

    void setupExecutionGraph();

    boost::filesystem::path m_logicFile;
    boost::filesystem::path m_sceneFile;

};

#endif // GRSF_Converters_RenderConverter_hpp


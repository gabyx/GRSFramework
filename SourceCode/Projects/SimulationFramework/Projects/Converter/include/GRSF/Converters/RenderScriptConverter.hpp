#ifndef RenderScriptConverter_hpp
#define RenderScriptConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <pugixml.hpp>

#include <boost/filesystem.hpp>

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Common/ApplicationCLOptionsConverter.hpp"

#include "GRSF/Common/SimpleLogger.hpp"
#include "GRSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GRSF/General/RenderData.hpp"
#include "GRSF/General/RenderExecutionGraph.hpp"

class RenderScriptConverter : public LogicScriptConverter<RenderExecutionGraph> {
public:

    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;

    using Base = LogicScriptConverter<RenderExecutionGraph>;

    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path outputDir,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path materialFile,
                  Renderer renderer);

    using RenderScriptGen = RenderExecutionGraph;

private:

    RenderData m_renderData;

    void setupGenerator();

    Renderer m_renderer;

};

#endif // RenderScriptConverter_hpp


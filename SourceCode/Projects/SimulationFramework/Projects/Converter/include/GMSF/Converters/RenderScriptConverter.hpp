#ifndef RenderScriptConverter_hpp
#define RenderScriptConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>

#include "GMSF/Common/LogDefines.hpp"
#include "GMSF/Common/TypeDefs.hpp"

#include "GMSF/Common/ApplicationCLOptionsConverter.hpp"

#include "GMSF/Common/SimpleLogger.hpp"
#include "GMSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GMSF/General/RenderData.hpp"
#include "GMSF/General/RenderScriptGenerator.hpp"

class RenderScriptConverter {
public:

    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES

    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path materialFile,
                  Renderer renderer);

    using RenderScriptGen = RenderScriptGenerator;

private:

    RenderData m_renderData;
    RenderScriptGen m_renderScriptGen;

    void loadGeometryCollection();
    void loadMaterialCollection();

    void convertFile(const boost::filesystem::path & f);

    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    boost::filesystem::path m_materialFile;
    boost::filesystem::path m_sceneFile;

    boost::filesystem::path m_outputFile;
    std::vector<boost::filesystem::path> m_inputFiles;
    Renderer m_renderer;

    unsigned int m_frameCounter;
};

#endif // RenderScriptConverter_hpp


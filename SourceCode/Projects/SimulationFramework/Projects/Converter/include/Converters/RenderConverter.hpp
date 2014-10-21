#ifndef RenderConverter_hpp
#define RenderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "ApplicationCLOptionsConverter.hpp"

#include "SimpleLogger.hpp"
#include "MultiBodySimFile.hpp"
#include "RenderConverterData.hpp"

class RenderConverter {
public:

    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES

    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path materialFile,
                  Renderer renderer);
private:

    RenderConverterData m_renderData;

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

#endif // RenderConverter_hpp


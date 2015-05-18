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
#include "GRSF/General/RenderScriptGenerator.hpp"

class RenderScriptConverter {
public:

    DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES

    using XMLNodeType = pugi::xml_node;
    using XMLNodeItType = pugi::xml_node_iterator;
    using XMLAttributeType = pugi::xml_attribute;


    using Renderer = typename ApplicationCLOptionsRenderer::Renderer;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path outputFile,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path materialFile,
                  Renderer renderer);

    using RenderScriptGen = RenderScriptGenerator;

private:

    using StateIndicesType = std::vector< std::streamoff >;

    RenderData m_renderData;
    RenderScriptGen m_renderScriptGen;

    void loadGeometryCollection();
    void loadMaterialCollection();

    /** \p uuid string is a hash for the file path to identify each frame where it came from!*/
    void convertFile(const boost::filesystem::path & f,
                     const std::string uuidString ,
                     StateIndicesType stateIndices = {} );



    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    boost::filesystem::path m_logicFile;
    boost::filesystem::path m_sceneFile;

    boost::filesystem::path m_outputFile;
    std::vector<boost::filesystem::path> m_inputFiles;
    Renderer m_renderer;

    unsigned int m_frameCounter;

    bool m_abort;
    void callbackAbort(){ m_abort = true; LOG(m_log, "---> Quitting ...:" <<std::endl);}
};

#endif // RenderScriptConverter_hpp


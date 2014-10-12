#ifndef RenderConverter_hpp
#define RenderConverter_hpp

#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <vector>

#include <boost/filesystem.hpp>
#include "SimpleLogger.hpp"
#include "CPUTimer.hpp"
#include "ProgressBarCL.hpp"
#include "MultiBodySimFile.hpp"


#include "LogDefines.hpp"




class RenderConverter{
public:


    using RenderEnum = ApplicationCLOptionsRenderer::RendererEnum;

    void convert( const std::vector<boost::filesystem::path> & inputFiles,
                  boost::filesystem::path sceneFile,
                  boost::filesystem::path materialFile,
                  boost::filesystem::path outputFile,
                  RendererEnum renderer)
    {
        m_renderer = renderer;
        m_outputFile = outputFile;
        m_inputFiles = inputFiles;
        m_sceneFile = sceneFile;
        m_materialFile = materialFile;
        auto log = outputFile.parent_path() / "RenderConverter.log";
        m_log = Logging::LogManager::getSingletonPtr()->createLog("RenderConverter",true,true,log);

        LOG(m_log, "---> RenderConverter started:" <<std::endl;);


        // First open the sim file
        for(auto file : m_inputFiles){
            convertFile(file);
        }

    }

private:

    void convertFile(const boost::filesystem::path & f){
        LOG(m_log, "---> Converting file:" << f << std::endl;);



    }

    MultiBodySimFile m_simFile;

    Logging::Log * m_log;

    boost::filesystem::path m_materialFile;
    boost::filesystem::path m_sceneFile;

    boost::filesystem::path m_outputFile;
    std::vector<boost::filesystem::path> m_inputFiles;
    RendererEnum m_renderer;
};

#endif // RenderConverter_hpp


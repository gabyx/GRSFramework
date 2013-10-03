#include <iostream>
#include <string>
#include "LogDefines.hpp"
#include "TypeDefs.hpp"
#include "ApplicationCLOptions.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"

#include "SimulationManager.hpp"


int main(int argc, char **argv) {
    // Parsing Input Parameters===================================
    new ApplicationCLOptions();
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);

    ApplicationCLOptions::getSingletonPtr()->checkArguments();

    // End Parsing =================================

    new Logging::LogManager();

    std::stringstream processFolder;
    processFolder <<  PROCESS_FOLDER_PREFIX;
    boost::filesystem::path localDirPath;

    localDirPath = ApplicationCLOptions::getSingletonPtr()->m_localDir[0];
    localDirPath /= processFolder.str();


    // Process static global members! (Singletons)
    new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath); //Creates path if it does not exist


    SimulationManager<GeneralConfig> mgr;

    mgr.setup(ApplicationCLOptions::getSingletonPtr()->m_sceneFile);
    mgr.startSim();


    system("pause");
    return 0;
}


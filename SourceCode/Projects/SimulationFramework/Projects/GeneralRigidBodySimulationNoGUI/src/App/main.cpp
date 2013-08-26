#include <iostream>
#include <string>
#include "TypeDefs.hpp"
#include "ApplicationCLOptions.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"

#include "SimulationManager.hpp"


int main(int argc, char **argv)
{
   // Parsing Input Parameters===================================
    new ApplicationCLOptions();
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);

    ApplicationCLOptions::getSingletonPtr()->checkArguments();

   // End Parsing =================================

   new Logging::LogManager();
   new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir);

   SimulationManager<GeneralConfig> mgr;

   mgr.setup(ApplicationCLOptions::getSingletonPtr()->m_sceneFile);
   mgr.startSim();


  system("pause");
  return 0;
}


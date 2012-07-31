#include <iostream>
#include <string>
#include "TypeDefs.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"

#include "SimulationManager.hpp"


int main(int argc, char **argv)
{
    // Parsing ===================================
    char * sceneFileName = NULL;
    for (int i = 1; i < argc; i++) {
        std::cout << argv[i] << std::endl;
        if (std::string(argv[i]) == "-s") {
            // We know the next argument *should* be the filename:
            sceneFileName = argv[i + 1];
            i++;
        }else{
          std::cout << "Wrong arguments specified!:" << std::endl <<"Options:" <<std::endl
          << " \t -s <SceneFilePath>"  <<std::endl;
          exit(-1);
        }
    }

    if(!sceneFileName){
        ERRORMSG("No scene file (.xml) supplied as argument: -s <SceneFilePath>");
    }

   // End Parsing =================================


   new Logging::LogManager();
   new FileManager();

   SimulationManager<GeneralConfig> mgr;

   mgr.setup(boost::filesystem::path(sceneFileName));
   mgr.startSim();


   system("pause");
  return 0;
}


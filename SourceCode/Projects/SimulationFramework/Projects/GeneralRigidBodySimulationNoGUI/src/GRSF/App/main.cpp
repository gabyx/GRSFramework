#include <iostream>
#include <string>

#include "GRSF/Common/Exception.hpp"

#include "GRSF/Common/LogDefines.hpp"
#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/ApplicationSignalHandler.hpp"
#include "GRSF/Common/ApplicationCLOptions.hpp"
#include "GRSF/Singeltons/FileManager.hpp"
#include "GRSF/Common/SimpleLogger.hpp"

#include "GRSF/States/SimulationManager/SimulationManager.hpp"

#include "ApproxMVBB/ComputeApproxMVBB.hpp"
#include "GRSF/Dynamics/Collision/Geometry/OOBB.hpp"

void callBackSIGINT(){
    std::cerr << "Caught signal SIGINT --> exit ..." << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char **argv) {

    ApplicationSignalHandler sigHandler( {SIGINT,SIGTERM,SIGUSR1,SIGUSR2} );
    sigHandler.registerCallback(SIGINT,callBackSIGINT,"callBackSIGINT");


    try{
        // Parsing Input Parameters===================================
        ApplicationCLOptions opts;
        ApplicationCLOptions::getSingleton().parseOptions(argc,argv);

        ApplicationCLOptions::getSingleton().checkArguments();
        ApplicationCLOptions::getSingleton().printArgs(std::cout);
        // End Parsing =================================

        //Create singleton logger
        Logging::LogManager logger;

        std::stringstream processFolder;
        processFolder <<  PROCESS_FOLDER_PREFIX;
        boost::filesystem::path localDirPath;

        localDirPath = ApplicationCLOptions::getSingleton().getLocalDirs()[0];
        localDirPath /= processFolder.str();


        // Process static global members! (Singletons)
        FileManager fileManger(ApplicationCLOptions::getSingleton().getGlobalDir(), localDirPath); //Creates path if it does not exist


        SimulationManager mgr;

        mgr.setup(ApplicationCLOptions::getSingleton().getSceneFile());
        mgr.startSim();


        // Do post processes at the end of the simulation
        //TODO
        for(auto & t : ApplicationCLOptions::getSingleton().getPostProcessTasks()){
            if(t->getName() == "bash"){
                t->execute();
            }
        }

    }catch(Exception& ex) {
        std::cerr << "Exception occured in process rank: "  << ex.what() <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_FAILURE);
    }catch(std::exception & ex){
        std::cerr << "std::exception occured: "  << ex.what() <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_FAILURE);
    }catch(...){
        std::cerr << "Unknown exception occured!" <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_FAILURE);
    }

    return 0;
}


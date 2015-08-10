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

    INSTANCIATE_UNIQUE_SINGELTON_CTOR(ApplicationSignalHandler,sigHandler, ( {SIGINT,SIGTERM,SIGUSR1,SIGUSR2} ) )
    sigHandler->registerCallback(SIGINT,callBackSIGINT,"callBackSIGINT");


    try{

        #ifndef NDEBUG
                std::cout << "GRSFramework Sim: build: ?, config: " << "debug" << std::endl;
        #else
                std::cout << "GRSFramework Sim: build: ?, config: " << "release" << std::endl;
        #endif

        // Parsing Input Parameters===================================
        INSTANCIATE_UNIQUE_SINGELTON(ApplicationCLOptions,opts)
        ApplicationCLOptions::getSingleton().parseOptions(argc,argv);

        ApplicationCLOptions::getSingleton().checkArguments();
        ApplicationCLOptions::getSingleton().printArgs(std::cout);
        // End Parsing =================================

        //Create singleton logger
        INSTANCIATE_UNIQUE_SINGELTON(Logging::LogManager, logger)

        std::stringstream processFolder;
        processFolder <<  PROCESS_FOLDER_PREFIX;
        boost::filesystem::path localDirPath;

        localDirPath = ApplicationCLOptions::getSingleton().getLocalDirs()[0];
        localDirPath /= processFolder.str();


        // Process static global members! (Singletons)
        INSTANCIATE_UNIQUE_SINGELTON_CTOR(FileManager, fileManger, (ApplicationCLOptions::getSingleton().getGlobalDir(), localDirPath) )

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


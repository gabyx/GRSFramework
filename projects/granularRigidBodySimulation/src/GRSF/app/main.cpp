// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <iostream>
#include <string>

#include "GRSF/common/Exception.hpp"

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/ApplicationSignalHandler.hpp"
#include "GRSF/common/ApplicationCLOptions.hpp"
#include "GRSF/singeltons/FileManager.hpp"
#include "GRSF/common/SimpleLogger.hpp"

#include "GRSF/states/simulationManager/SimulationManager.hpp"

#include "ApproxMVBB/ComputeApproxMVBB.hpp"
#include "GRSF/dynamics/collision/geometry/OOBB.hpp"

void callBackExit(int signum){
    // http://www.cons.org/cracauer/sigint.html
    // set sigint handler to nothing
    // and kill ourself
//    signal(SIGINT, SIG_DFL);
//    kill(getpid(),SIGINT);
    std::cerr << "GRSFramework Sim: received signal: " << signum << " -> exit ..." << std::endl;
    GRSF_THROW_SIGNALEXCEPTION("GRSFramework Sim: received signal: " << signum << " -> exit ...");
}

int main(int argc, char **argv) {

    INSTANCIATE_UNIQUE_SINGELTON_CTOR(ApplicationSignalHandler,sigHandler, ( {SIGINT,SIGUSR2,SIGPIPE} ) )
    sigHandler->registerCallback({SIGINT,SIGUSR2,SIGPIPE},callBackExit,"callBackExit");

    try{

        std::cerr << "GRSFramework Sim: version: " GRSF_VERSION_STRING
        #ifndef NDEBUG
                " | config: debug" << std::endl;
        #else
                " | config: release" << std::endl;
        #endif

        // Parsing Input Parameters===================================
        INSTANCIATE_UNIQUE_SINGELTON(ApplicationCLOptions,opts)
        ApplicationCLOptions::getSingleton().parseOptions(argc,argv);

        ApplicationCLOptions::getSingleton().checkArguments();
        ApplicationCLOptions::getSingleton().printArgs(std::cerr);
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
    }catch(SignalException& ex) {
        std::cerr << "SignalException occured: "  << ex.what() <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        exit(EXIT_SUCCESS);
    }catch(Exception& ex) {
        std::cerr << "Exception occured: "  << ex.what() <<std::endl;
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


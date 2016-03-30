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
#include <memory>

#include <mpi.h>


#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/ApplicationSignalHandler.hpp"
#include "GRSF/common/ApplicationCLOptions.hpp"
#include "GRSF/singeltons/FileManager.hpp"
#include "GRSF/common/SimpleLogger.hpp"
#include "GRSF/common/RedirectOutput.hpp"

#include "GRSF/dynamics/general/MPIDataTypes.hpp"
#include "GRSF/states/simulationManager/SimulationManagerMPI.hpp"


void start( int argc, char **argv ){

        // Setup global communicators

        INSTANCIATE_UNIQUE_SINGELTON( MPILayer::MPIGlobalCommunicators , globalComm )

        // Setup global types and commit them
        MPILayer::DataTypes::commitAll();
        MPILayer::ReduceFunctions::createAll();

         // Add the process rank to the Global File Path for this Process...
        int my_rank;
        MPI_Comm_rank(MPI_COMM_WORLD,&my_rank);
        int nProcesses;
        MPI_Comm_size(MPI_COMM_WORLD,&nProcesses);

        // Parsing Input Parameters===================================
        INSTANCIATE_UNIQUE_SINGELTON(ApplicationCLOptions,opts)

        ApplicationCLOptions::getSingleton().parseOptions(argc,argv);
        ApplicationCLOptions::getSingleton().checkArguments();
        if(my_rank == 0){

            std::cout << "GRSFramework SimMPI: version: " GRSF_VERSION_STRING
            #ifndef NDEBUG
                    " | config: " << "debug" << std::endl;
            #else
                    " | config: " << "release" << std::endl;
            #endif

            ApplicationCLOptions::getSingleton().printArgs(std::cout);

        }
        // End Parsing =================================


        std::stringstream processFolder;
        processFolder << PROCESS_FOLDER_PREFIX << my_rank;
        boost::filesystem::path localDirPath;

        //Calculate the directory where the processes have theis local dir
        {
            auto & localDirs = ApplicationCLOptions::getSingleton().getLocalDirs();
            unsigned int groups = nProcesses / localDirs.size();
            unsigned int index = std::min( (unsigned int)(my_rank/ groups), (unsigned int)localDirs.size()-1);
            localDirPath = localDirs[index];
            localDirPath /= processFolder.str();
        }

        // Rank 0 makes the FileManager first( to ensure that all folders are set up properly)
        std::unique_ptr<FileManager> fileManager;
        if(my_rank == 0){
            fileManager.reset(new FileManager(ApplicationCLOptions::getSingleton().getGlobalDir(), localDirPath)); //Creates path if it does not exist
            MPI_Barrier(MPI_COMM_WORLD);
        }
        else{
            MPI_Barrier(MPI_COMM_WORLD);
            //These do not create paths anymore because rank 0 has already made the stuff
            fileManager.reset(new FileManager(ApplicationCLOptions::getSingleton().getGlobalDir(), localDirPath));
        }

        // Redirect std::cerr to Global file:
        boost::filesystem::path file = FileManager::getSingleton().getGlobalDirectoryPath();
        std::stringstream name;
        name << "MPIError_Rank" << my_rank << ".log";
        file /= name.str();
        std::fstream f;
        f.open(file.string().c_str(), std::ios_base::trunc | std::ios_base::out);
        const RedirectOutputs _(f, std::cerr);

        // Catch Exceptions into ErrorLog
        try{

            INSTANCIATE_UNIQUE_SINGELTON( Logging::LogManager, logger )

            // Construct the communicator for the Simulation
            MPILayer::MPIGlobalCommunicators::getSingleton().addCommunicator(MPILayer::MPICommunicatorId::SIM_COMM,MPI_COMM_WORLD);

            // Only Simulation Processes enter this region:
                SimulationManagerMPI mgr;
                mgr.setup(ApplicationCLOptions::getSingleton().getSceneFile());
                mgr.startSim();
            // =============================================

            // Only other process (not needed yet) enter this region


            // =============================================

           // Do post processes at the end of the simulation
            //TODO
            for(auto & t : ApplicationCLOptions::getSingleton().getPostProcessTasks()){
                if(t->getName() == "bash"){
                    t->execute();
                }
            }

        }catch(Exception& ex) {
            int rank; MPI_Comm_rank(MPI_COMM_WORLD,&rank);
            std::cerr << "Exception occured in process rank: " << rank << std::endl << ex.what() <<std::endl;
            std::cerr << "Exiting ..." << std::endl;
            MPI_Abort(MPI_COMM_WORLD,-1);
        }catch(std::exception & ex){
            int rank; MPI_Comm_rank(MPI_COMM_WORLD,&rank);
            std::cerr << "Std::exception occured in process rank: " << rank << std::endl << ex.what() <<std::endl;
            std::cerr << "Exiting ..." << std::endl;
            MPI_Abort(MPI_COMM_WORLD,-1);
        }catch(...){
            int rank; MPI_Comm_rank(MPI_COMM_WORLD,&rank);
            std::cerr << "Unknown exception occured in process rank: " << rank <<std::endl;
            std::cerr << "Exiting ..." << std::endl;
            MPI_Abort(MPI_COMM_WORLD,-1);
        }

        MPILayer::ReduceFunctions::freeAll();
        MPILayer::DataTypes::freeAll();

}


void callBackSignalAndExit(int signum){
    std::cerr << "GRSFramework Sim MPI: received signal: " << signum << " -> exit ..." << std::endl;
    // http://www.cons.org/cracauer/sigint.html
    // set sigint handler to nothing
    // and kill ourself
    signal(SIGINT, SIG_DFL);
    kill(getpid(),SIGINT);
}

int main(int argc, char **argv) {


    INSTANCIATE_UNIQUE_SINGELTON_CTOR(ApplicationSignalHandler,sigHandler, ( {SIGINT,SIGUSR2} ) )
    sigHandler->registerCallback({SIGINT,SIGUSR2},callBackSignalAndExit,"callBackSignalAndExit");


    // Start MPI =================================
    MPI_Init(&argc, &argv);


    try{
        start(argc,argv);

    }catch(Exception& ex) {
        std::cerr << "Exception occured: "  << ex.what() <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        MPI_Abort(MPI_COMM_WORLD,-1);
    }catch(std::exception & ex){
        std::cerr << "std::exception occured: "  << ex.what() <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        MPI_Abort(MPI_COMM_WORLD,-1);
    }catch(...){
        std::cerr << "Unknown exception occured!" <<std::endl;
        std::cerr << "Exiting ..." << std::endl;
        MPI_Abort(MPI_COMM_WORLD,-1);
    }


    // Finalize MPI =================================
    MPI_Finalize();

    return 0;
}


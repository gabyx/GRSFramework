#include <iostream>
#include <csignal>
#include <string>

#include <mpi.h>

#include "TypeDefs.hpp"
#include "ApplicationCLOptions.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"
#include "RedirectOutput.hpp"
#include "SimulationManagerMPI.hpp"

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
void signal_callback_handler(int signum)
{
   // Cleanup and close up stuff here
   // Terminate program
   ERRORMSG("---> Caught signal: " << signum << " in signal handler!" << std::endl);
}


int main(int argc, char **argv) {


    signal(SIGINT, signal_callback_handler);
    signal(SIGTERM, signal_callback_handler);
    signal(SIGUSR1, signal_callback_handler);
    signal(SIGUSR2, signal_callback_handler);

    // Start MPI =================================
    MPI_Init(&argc, &argv);

    // Scope that all stuff is deconstructed before MPI FINALIZE IS CALLED
    {

        MPILayer::MPIGlobalCommunicators globalComm;


         // Add the process rank to the Global File Path for this Process...
        int my_rank;
        MPI_Comm_rank(MPI_COMM_WORLD,&my_rank);
        int nProcesses;
        MPI_Comm_size(MPI_COMM_WORLD,&nProcesses);


        // Parsing Input Parameters===================================
        ApplicationCLOptions opts;
        ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);
        ApplicationCLOptions::getSingletonPtr()->checkArguments();
        if(my_rank == 0){
            ApplicationCLOptions::getSingletonPtr()->printArgs(std::cout);
        }
        // End Parsing =================================


        std::stringstream processFolder;
        processFolder << PROCESS_FOLDER_PREFIX << my_rank;
        boost::filesystem::path localDirPath;

        //Calculate the directory where the processes have theis local dir
        {
            unsigned int groups = nProcesses / ApplicationCLOptions::getSingletonPtr()->m_localDirs.size();
            unsigned int index = std::min( (unsigned int)(my_rank/ groups), (unsigned int)ApplicationCLOptions::getSingletonPtr()->m_localDirs.size()-1);
            localDirPath = ApplicationCLOptions::getSingletonPtr()->m_localDirs[index];
            localDirPath /= processFolder.str();
        }

        // Rank 0 makes the FileManager first( to ensure that all folders are set up properly)
        FileManager * fileManager;
        if(my_rank == 0){
            fileManager = new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath); //Creates path if it does not exist
            MPI_Barrier(MPI_COMM_WORLD);
        }
        else{
            MPI_Barrier(MPI_COMM_WORLD);
            //These do not create paths anymore because rank 0 has already made the stuff
            fileManager = new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath);
        }

        Logging::LogManager logger;

        // Redirect std::cerr to Global file:
        boost::filesystem::path file = FileManager::getSingletonPtr()->getGlobalDirectoryPath();
        std::stringstream name;
        name << "MPIError_Rank" << my_rank << ".log";
        file /= name.str();
        std::fstream f;
        f.open(file.string().c_str(), std::ios_base::trunc | std::ios_base::out);
        const RedirectOutputs _(f, std::cerr);


        // Construct the communicator for the Simulation
        MPILayer::MPIGlobalCommunicators::getSingletonPtr()->addCommunicator(MPILayer::MPICommunicatorId::SIM_COMM,MPI_COMM_WORLD);

        // Only Simulation Processes enter this region:
            SimulationManagerMPI mgr;
            mgr.setup(ApplicationCLOptions::getSingletonPtr()->m_sceneFile);
            mgr.startSim();
        // =============================================

        // Only other process (not needed yet) enter this region


        // =============================================

       // Do post processes at the end of the simulation
        //TODO
        auto & tasks = ApplicationCLOptions::getSingletonPtr()->m_postProcessTasks;
        for(auto it = tasks.begin(); it != tasks.end(); it++){
            if((*it)->getName() == "bash"){
                (*it)->execute();
            }
        }


        delete fileManager;

    } // SCOPE


    // Finalize MPI =================================
    MPI_Finalize();

    return 0;
}


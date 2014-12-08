#include <iostream>
#include <csignal>
#include <string>
#include <memory>

#include <mpi.h>


#include "GMSF/Common/TypeDefs.hpp"
#include "GMSF/Common/ApplicationCLOptions.hpp"
#include "GMSF/Singeltons/FileManager.hpp"
#include "GMSF/Common/SimpleLogger.hpp"
#include "GMSF/Common/RedirectOutput.hpp"

#include "GMSF/Dynamics/General/MPIDataTypes.hpp"
#include "GMSF/States/SimulationManager/SimulationManagerMPI.hpp"

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
void signal_callback_handler(int signum)
{
   // Cleanup and close up stuff here
   // Terminate program
   std::string signal;

   switch(signum){
       case SIGINT:
            signal = "SIGINT";  break;
       case SIGTERM:
            signal = "SIGTERM"; break;
       case SIGUSR1:
            signal = "SIGUSR1"; break;
       case SIGUSR2:
            signal = "SIGUSR2"; break;
   }

   std::cerr << "---> Caught signal: " << signal << " in signal handler! Exiting..." << std::endl;
   exit(EXIT_FAILURE);
}


void start( int argc, char **argv ){

        // Setup global communicators
        MPILayer::MPIGlobalCommunicators globalComm;

        // Setup global types and commit them
        MPILayer::DataTypes::commitAll();
        MPILayer::ReduceFunctions::createAll();

         // Add the process rank to the Global File Path for this Process...
        int my_rank;
        MPI_Comm_rank(MPI_COMM_WORLD,&my_rank);
        int nProcesses;
        MPI_Comm_size(MPI_COMM_WORLD,&nProcesses);

        // Parsing Input Parameters===================================
        ApplicationCLOptions opts;
        ApplicationCLOptions::getSingleton().parseOptions(argc,argv);
        ApplicationCLOptions::getSingleton().checkArguments();
        if(my_rank == 0){
            ApplicationCLOptions::getSingleton().printArgs(std::cout);
        }
        // End Parsing =================================


        std::stringstream processFolder;
        processFolder << PROCESS_FOLDER_PREFIX << my_rank;
        boost::filesystem::path localDirPath;

        //Calculate the directory where the processes have theis local dir
        {
            unsigned int groups = nProcesses / ApplicationCLOptions::getSingleton().m_localDirs.size();
            unsigned int index = std::min( (unsigned int)(my_rank/ groups), (unsigned int)ApplicationCLOptions::getSingleton().m_localDirs.size()-1);
            localDirPath = ApplicationCLOptions::getSingleton().m_localDirs[index];
            localDirPath /= processFolder.str();
        }

        // Rank 0 makes the FileManager first( to ensure that all folders are set up properly)
        std::unique_ptr<FileManager> fileManager;
        if(my_rank == 0){
            fileManager.reset(new FileManager(ApplicationCLOptions::getSingleton().m_globalDir, localDirPath)); //Creates path if it does not exist
            MPI_Barrier(MPI_COMM_WORLD);
        }
        else{
            MPI_Barrier(MPI_COMM_WORLD);
            //These do not create paths anymore because rank 0 has already made the stuff
            fileManager.reset(new FileManager(ApplicationCLOptions::getSingleton().m_globalDir, localDirPath));
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

            Logging::LogManager logger;

            // Construct the communicator for the Simulation
            MPILayer::MPIGlobalCommunicators::getSingleton().addCommunicator(MPILayer::MPICommunicatorId::SIM_COMM,MPI_COMM_WORLD);

            // Only Simulation Processes enter this region:
                SimulationManagerMPI mgr;
                mgr.setup(ApplicationCLOptions::getSingleton().m_sceneFile);
                mgr.startSim();
            // =============================================

            // Only other process (not needed yet) enter this region


            // =============================================

           // Do post processes at the end of the simulation
            //TODO
            auto & tasks = ApplicationCLOptions::getSingleton().m_postProcessTasks;
            for(auto it = tasks.begin(); it != tasks.end(); it++){
                if((*it)->getName() == "bash"){
                    (*it)->execute();
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

int main(int argc, char **argv) {


    signal(SIGINT, signal_callback_handler);
    signal(SIGTERM, signal_callback_handler);
    signal(SIGUSR1, signal_callback_handler);
    signal(SIGUSR2, signal_callback_handler);

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


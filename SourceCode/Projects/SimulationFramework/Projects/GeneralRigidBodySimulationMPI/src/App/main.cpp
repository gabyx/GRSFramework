#include <iostream>
#include <string>

#include <mpi.h>

#include "TypeDefs.hpp"
#include "ApplicationCLOptions.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"
#include "RedirectOutput.hpp"
#include "SimulationManagerMPI.hpp"


int main(int argc, char **argv) {




    // Start MPI =================================
    MPI_Init(&argc, &argv);

     // Add the process rank to the Global File Path for this Process...
    int my_rank;
    MPI_Comm_rank(MPI_COMM_WORLD,&my_rank);
    int nProcesses;
    MPI_Comm_size(MPI_COMM_WORLD,&nProcesses);


    // Parsing Input Parameters===================================
    new ApplicationCLOptions();
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);
    ApplicationCLOptions::getSingletonPtr()->checkArguments();
    if(my_rank == 0){
        ApplicationCLOptions::getSingletonPtr()->printArgs();
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
    if(my_rank == 0){
        new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath); //Creates path if it does not exist
        MPI_Barrier(MPI_COMM_WORLD);
    }
    else{
        MPI_Barrier(MPI_COMM_WORLD);
        //These do not create paths anymore because rank 0 has already made the stuff
        new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath);
    }

    new Logging::LogManager();

    // Redirect std::cerr to Global file:
    boost::filesystem::path file = FileManager::getSingletonPtr()->getGlobalDirectoryPath();
    file /= "GlobalMPIError.log";
    std::fstream f;
    f.open(file.string().c_str(), std::ios_base::trunc | std::ios_base::out);
    const RedirectOutputs _(f, std::cerr);


    SimulationManagerMPI<GeneralConfig> mgr;

    mgr.setup(ApplicationCLOptions::getSingletonPtr()->m_sceneFile);


    mgr.startSim();

    // Finalize MPI =================================
    MPI_Finalize();

    return 0;
}


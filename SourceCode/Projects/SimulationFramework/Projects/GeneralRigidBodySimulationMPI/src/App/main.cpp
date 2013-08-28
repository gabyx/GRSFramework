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


    // Parsing Input Parameters===================================
    new ApplicationCLOptions();
    ApplicationCLOptions::getSingletonPtr()->parseOptions(argc,argv);
    ApplicationCLOptions::getSingletonPtr()->checkArguments();
    // End Parsing =================================


    // Add the process rank to the Global File Path for this Process...
    int my_rank;
    MPI_Comm_rank(MPI_COMM_WORLD,&my_rank);
    int nProcesses;
    MPI_Comm_size(MPI_COMM_WORLD,&nProcesses);

    std::stringstream processFolder;
    processFolder << "Process_" << my_rank;
    boost::filesystem::path localDirPath;

    localDirPath = ApplicationCLOptions::getSingletonPtr()->m_globalDir;
    localDirPath /= processFolder.str();


    // Process static global members! (Singletons)
    new FileManager(ApplicationCLOptions::getSingletonPtr()->m_globalDir, localDirPath); //Creates path if it does not exist
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


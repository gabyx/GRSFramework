#include <iostream>
#include <string>

#include <mpi.h>

#include "TypeDefs.hpp"
#include "FileManager.hpp"
#include "SimpleLogger.hpp"

#include "SimulationManagerMPI.hpp"


int main(int argc, char **argv) {

    // Start MPI =================================
    MPI_Init(&argc, &argv);


    // Parsing Input Parameters===================================
    char * sceneFilePath = NULL;
    char * globalFilePathChar = NULL;

    for (int i = 1; i < argc; i++) {
        std::cout << argv[i] << std::endl;
        if (std::string(argv[i]) == "-s") {
            // We know the next argument *should* be the filename:
            sceneFilePath = argv[i + 1];
            i++;
            std::cout << " SceneFile Arg: " << sceneFilePath <<std::endl;
        }else if(std::string(argv[i]) == "-p"){
          globalFilePathChar  = argv[i + 1];
          i++;
          std::cout << " GlobalFilePath Arg: " << globalFilePathChar <<std::endl;
        } else {
            std::cout << "Wrong arguments specified!:" << std::endl <<"Options:" <<std::endl
                      << " \t -s <SceneFilePath>"  <<std::endl;
            exit(-1);
        }
    }

    if(!sceneFilePath){
        ERRORMSG("No scene file (.xml) supplied as argument: -s <SceneFilePath>");
    }



    // End Parsing =================================


    // Add the process rank to the Global File Path for this Process...
    int my_rank;
    MPI_Comm_rank(MPI_COMM_WORLD,&my_rank);
    int nProcesses;
    MPI_Comm_size(MPI_COMM_WORLD,&nProcesses);

    std::stringstream processFolder;
    processFolder << "Process_" << my_rank << std::endl;
    boost::filesystem::path globalFilePath;
    if(globalFilePathChar){
        globalFilePath = boost::filesystem::path(globalFilePathChar);
    }
    globalFilePath /= processFolder.str();


    // Process static global members! (Singletons)
    new FileManager(globalFilePath); //Creates path if it does not exist
    new Logging::LogManager();



    SimulationManagerMPI<GeneralConfig> mgr;

    mgr.setup(boost::filesystem::path(sceneFilePath));



    // Finalize MPI =================================
    MPI_Finalize();

    return 0;
}


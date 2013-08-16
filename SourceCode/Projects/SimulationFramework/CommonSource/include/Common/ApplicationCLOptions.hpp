#ifndef ApplicationCLOptions_hpp
#define ApplicationCLOptions_hpp

#include <string>
#include <boost/filesystem.hpp>
#include "Singleton.hpp"


/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions: public Utilities::Singleton<ApplicationCLOptions>{
public:

    boost::filesystem::path m_globalDir;
    boost::filesystem::path m_sceneFile;

    void parseOptions(int argc, char **argv){
        char * sceneFilePathChar = NULL;
        char * globalFilePathChar = NULL;

        for (int i = 1; i < argc; i++) {
            if (std::string(argv[i]) == "-s") {
                // We know the next argument *should* be the filename:
                if(i + 1 >= argc){
                    printErrorNoArg("-s");
                }
                sceneFilePathChar = argv[i + 1];
                i++;
                std::cout << " SceneFile Arg: " << sceneFilePathChar <<std::endl;
            }else if(std::string(argv[i]) == "-p"){
                if(i + 1 >= argc){
                    printErrorNoArg("-p");
                }
              globalFilePathChar  = argv[i + 1];
              i++;
              std::cout << " GlobalFilePath Arg: " << globalFilePathChar <<std::endl;
            } else {
                std::cout << "Wrong option: '" <<std::string(argv[i]) << "'" << std::endl;
                printHelp();
            }
        }

        if(sceneFilePathChar){
            m_sceneFile = boost::filesystem::path(std::string(sceneFilePathChar));
        }

        if(globalFilePathChar){
           m_globalDir =  boost::filesystem::path(std::string(globalFilePathChar));
        }


    }

    void checkArguments(){
        if(m_sceneFile.empty()){
            ERRORMSG("No scene file (.xml) supplied as argument: -s [SceneFilePath]");
        }
    }

    private:

    void printErrorNoArg(std::string arg){
        std::cout << "Wrong options specified for arguement: '" << arg <<"'"<< std::endl;
        printHelp();
        exit(-1);
    }

    void printHelp(){
        std::cout << "Help for the Application:" << std::endl <<"Options:" <<std::endl
                              << " \t -s [SceneFilePath] (This is a .xml file)" <<std::endl
                              << " \t -p [GlobalFilePath]"  <<std::endl;
                    exit(-1);
    }
};




#endif

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

    boost::filesystem::path m_localDir = "./";
    boost::filesystem::path m_globalDir = "./";
    boost::filesystem::path m_sceneFile;

    void parseOptions(int argc, char **argv){
        char * sceneFilePathChar = NULL;
        char * globalFilePathChar = NULL;
        char * localFilePathChar = NULL;

        for (int i = 1; i < argc; i++) {
            if (std::string(argv[i]) == "-s") {
                // We know the next argument *should* be the filename:
                if(i + 1 >= argc){
                    printErrorNoArg("-s");
                }
                sceneFilePathChar = argv[i + 1];
                i++;
                std::cout << " SceneFile Arg: " << sceneFilePathChar <<std::endl;
            }else if(std::string(argv[i]) == "-pg"){
                if(i + 1 >= argc){
                    printErrorNoArg("-pg");
                }
              globalFilePathChar  = argv[i + 1];
              i++;
              std::cout << " GlobalFilePath Arg: " << globalFilePathChar <<std::endl;
            }
            else if(std::string(argv[i]) == "-pl"){
                if(i + 1 >= argc){
                    printErrorNoArg("-pl");
                }
              localFilePathChar  = argv[i + 1];
              i++;
              std::cout << " LocalFilePath Arg: " << localFilePathChar <<std::endl;
            }
            else if(std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help" ){
                printHelp();
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
            m_localDir =  m_globalDir;
        }

        if(localFilePathChar){
            m_localDir =  boost::filesystem::path(std::string(localFilePathChar));
        }


    }

    void checkArguments(){
        if(m_sceneFile.empty()){
            printHelp();
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
                              << " \t -s [SceneFilePath] \n"
                              <<            "\t\t This is a .xml file for the scene, essential \n"
                              <<            "\t\t for CLI version, in GUI version: \"SceneFile.xml\" is the default file \n"
                              << " \t -pg [GlobalDirectoryPath] (optional) \n"
                              <<            "\t\t This is the global directory path. (no slash at the end, boost::create_directory bug!)\n"
                              << " \t -pl [LocalDirectoryPath] (optional) \n"
                              <<            "\t\t This is the local directory for each processes output, \n"
                              <<            "\t\t if not specified the local directory is the same as the global directory.\n"
                              <<            "\t\t (no slash at the end, boost::create_directory bug!)\n"
                              << " \t -h|--help \n"
                              <<            "\t\t Prints this help" <<std::endl;
                    exit(-1);
    }
};




#endif

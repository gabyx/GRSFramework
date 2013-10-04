#ifndef ApplicationCLOptions_hpp
#define ApplicationCLOptions_hpp


#include <string>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>


#include "Singleton.hpp"
#include "CommonFunctions.hpp"

/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions: public Utilities::Singleton<ApplicationCLOptions> {
public:

    std::vector<boost::filesystem::path> m_localDirs;
    boost::filesystem::path m_globalDir = "./";
    boost::filesystem::path m_sceneFile;

    void parseOptions(int argc, char **argv) {
        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);

        std::vector<std::string> args;

        ops >> Option("s",args);
        Utilities::printVector(std::cout, args.begin(), args.end(), ", ");
        exit(-1);

        for (int i = 1; i < argc; i++) {
            if (std::string(argv[i]) == "-s") {
                // We know the next argument *should* be the filename:
                if(i + 1 >= argc) {
                    printErrorNoArg("-s");
                }
                m_sceneFile = boost::filesystem::path(std::string(argv[i + 1]));
                i++;

            } else if(std::string(argv[i]) == "-pg") {
                if(i + 1 >= argc) {
                    printErrorNoArg("-pg");
                }
                m_globalDir =  boost::filesystem::path(std::string(argv[i + 1]));
                i++;
            } else if(std::string(argv[i]) == "-pl") {
                //Process all further local paths
                int j = i+1;
                while(true) {
                    if(j >= argc) {
                        if(j==i+1) {
                            printErrorNoArg("-pl");
                        }
                        j--; i = j; break;
                    }

                    // we can check the argument, if we are at a new command break
                    if( std::string(argv[j])[0] =='-') {
                        j--; i = j; break;
                    }

                    m_localDirs.push_back(  boost::filesystem::path(std::string(argv[j])) );
                    j++;
                }

            } else if(std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help" ) {
                printHelp();
            } else {
                std::cout << "Wrong option: '" <<std::string(argv[i]) << "'" << std::endl;
                printHelp();
            }
        }

        if(m_localDirs.size()==0){
            m_localDirs.push_back("./");
        }


    }

    void printArgs(){
        std::cout << " SceneFile Arg: " << m_sceneFile <<std::endl;
        std::cout << " GlobalFilePath Arg: " << m_globalDir <<std::endl;
        std::cout << " LocalFilePaths Args: ";
        Utilities::printVector(std::cout, m_localDirs.begin(), m_localDirs.end(), std::string(", "));
        std::cout << std::endl;
    }

    void checkArguments() {
        if(m_sceneFile.empty()) {
            printHelp();
            ERRORMSG("No scene file (.xml) supplied as argument: -s [SceneFilePath]");
        }else{
            if(! boost::filesystem::exists(m_sceneFile)){
                printHelp();
                ERRORMSG("Scene file supplied as argument: " << m_sceneFile << " does not exist!" );
            }
        }
    }

private:

    void printErrorNoArg(std::string arg) {
        std::cout << "Wrong options specified for arguement: '" << arg <<"'"<< std::endl;
        printHelp();
        exit(-1);
    }

    void printHelp() {
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
                  <<            "\t\t This can also be a list of directories (space delimited), which is \n"
                  <<            "\t\t distributed linearly over all participating processes.\n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help" <<std::endl;
                  exit(-1);
              }
              };




#endif

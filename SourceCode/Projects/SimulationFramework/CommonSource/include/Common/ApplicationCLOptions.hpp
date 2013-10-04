#ifndef ApplicationCLOptions_hpp
#define ApplicationCLOptions_hpp


#include <string>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>


#include "Singleton.hpp"
#include "CommonFunctions.hpp"

/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions: public Utilities::Singleton<ApplicationCLOptions> {
public:

    struct PostProcessTask{
        PostProcessTask(std::string name):m_name(name){};
        std::string m_name;
        std::vector<std::string> m_options;
    };

    std::vector<boost::filesystem::path> m_localDirs;
    boost::filesystem::path m_globalDir = "./";
    boost::filesystem::path m_sceneFile;

    // If I would like to have some posst process tasks going on afterwards
    // Like bash command and so one
    // This is not needed sofar in MPI mode, because I can do this with mpirun -npernode 1 command and so on which lets me postprocess files
    // per node on the cluster or
    std::vector<PostProcessTask> m_postProcessTasks;

    void parseOptions(int argc, char **argv) {
        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);
        ops.exceptions_all();
        try {

            if( ops >> OptionPresent('h',"help")) {
                printHelp();
            }

            std::string s;
            ops >> Option('s',s);
            m_sceneFile = boost::filesystem::path(s);



            if( ops >> OptionPresent('g',"global-path")) {
                ops >> Option('g',"global-path",s);
                m_globalDir = boost::filesystem::path(s);
            }

            if( ops >> OptionPresent('l',"local-path")) {
                std::vector<std::string> svec;
                ops >> Option('l',"local-path",svec);
                for(auto it = svec.begin(); it != svec.end(); it++){
                    m_localDirs.push_back(*it);
                }
            }

//            if( ops >> OptionPresent('p', "post-process")) {
//                std::vector<std::string> svec;
//                ops >> Option('p',"post-process",svec);
//
//                std::vector<int> splitIdx;
//                int nextArgIdx;
//                for(int i = 0; i < svec.size; it++){
//                    if(svec[i] == "bash"){
//                        // has 2 arguments [int|all] and string which is the bash command!
//                       splitIdx.push_back(i);
//                       nextArgIdx = i + 3;
//                    }else if( *it == "copy-local-to-global"){
//                        //has no arguments
//                        nextArgIdx = i + 1;
//                        m_postProcessTasks.push_back("copy-local-to-global");
//                    }else{
//                        if(i >= nextArg){
//                            std::cerr <<"Postprocess Argument: " << svec[i] << " not known!"
//                            printHelp();
//                        }
//                    }
//                }
//
//            }

        } catch(GetOpt::GetOptEx ex) {
            std::cerr <<"Exception occured in parsing CMD args:\n" << std::endl;
            printHelp();
        }


        if(m_localDirs.size()==0) {
            m_localDirs.push_back("./");
        }


    }

    void printArgs() {
        std::cout << " SceneFile Arg: " << m_sceneFile <<std::endl;
        std::cout << " GlobalFilePath Arg: " << m_globalDir <<std::endl;
        std::cout << " LocalFilePaths Args: ";
        Utilities::printVector(std::cout, m_localDirs.begin(), m_localDirs.end(), std::string(", "));
        std::cout << std::endl;
    }

    void checkArguments() {
        if(m_sceneFile.empty()) {
            std::cerr  << "No scene file (.xml) supplied as argument: -s [SceneFilePath]" << std::endl;
            printHelp();
        } else {
            if(! boost::filesystem::exists(m_sceneFile)) {
                std::cerr  << "Scene file supplied as argument: " << m_sceneFile << " does not exist!"<< std::endl;
                printHelp();
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

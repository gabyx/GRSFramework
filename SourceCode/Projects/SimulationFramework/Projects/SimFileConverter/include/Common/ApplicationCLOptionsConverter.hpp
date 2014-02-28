#ifndef ApplicationCLOptionsConverter_hpp
#define ApplicationCLOptionsConverter_hpp


#include <string>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>

#include "AssertionDebug.hpp"
#include "Singleton.hpp"
#include "CommonFunctions.hpp"



/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions: public Utilities::Singleton<ApplicationCLOptions> {
public:

    std::vector<boost::filesystem::path> m_inputFiles;

    boost::filesystem::path m_outputFile;



    enum class Task: unsigned int{
        UNDEFINED = 0,
        JOIN = 1
    };
    Task m_task;


    void parseOptions(int argc, char **argv) {
        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);
        ops.exceptions_all();
        try {

            if( ops >> OptionPresent('h',"help")) {
                printHelp();
            }

            m_inputFiles.clear();
            ops >> Option('i',"input",m_inputFiles);
            // Clear all empty paths
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); ){
                    if(it->empty()){
                        it=m_inputFiles.erase(it);
                    }
                    else{
                        it++;
                    }
            }

            std::string task;
            ops >> Option('t',"task",task);
            if(task == "join"){

                m_task = Task::JOIN;
                ops >> Option('o',"output",m_outputFile);

            }else{
                std::cerr <<"Exception occured in parsing task arg" << std::endl;
                printHelp();
            }



        } catch(GetOpt::GetOptEx ex) {
            std::cerr <<"Exception occured in parsing args\n" << std::endl;
            printHelp();
        }

        if (ops.options_remain()){
            std::cerr <<"Some unexpected options where given!" << std::endl;
            printHelp();
        }

    }

    void printArgs(std::ostream & s){
        s << " Input Files Arg: ";
        Utilities::printVector(s, m_inputFiles.begin(), m_inputFiles.end(), std::string(" , "));
        s << std::endl;
        s << " Output File Arg: " << m_outputFile <<std::endl;
        s << " Task Arg: ";
        if(m_task == Task::JOIN){
            s << "join";
        }else{
            s << "undefined";
        }
        s<<std::endl;
    }

    void checkArguments() {

        if(m_inputFiles.empty()) {
            std::cerr  << "No input files supplied!" << std::endl;
            printHelp();
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {
                    std::cerr  << "Input file supplied as argument: " << *it << " does not exist!"<< std::endl;
                    printHelp();
                }
            }

        }

        if(m_task == Task::JOIN){
            if(m_outputFile.empty()){
                std::cerr  << "No output file supplied!" << std::endl;
                printHelp();
            }
            if(boost::filesystem::exists(m_outputFile)) {
                    std::cerr  << "Input file supplied as argument: " << m_outputFile << " does already exist (no overwrite is allowed)!"<< std::endl;
                    printHelp();
            }
        }

    }

private:

    void printErrorNoArg(std::string arg) {
        std::cerr << "Wrong options specified for arguement: '" << arg <<"'"<< std::endl;
        printHelp();
        exit(EXIT_FAILURE);
    }

    void printHelp() {
        std::cerr << "Help for the Application:" << std::endl <<"Options:" <<std::endl
                  << " \t -i|--input <path1> <path2> ... \n"
                  <<            "\t\t <path1> <path2> ... : These are multiple space delimited input sim file oaths which are processed \n"
                  << " \t -t|--task join (optional) \n"
                  <<            "\t\t This describes the task:\n"
                  <<            "\t\t\t 'join': Joins the multiple sim files together into one file\n"
                  << " \t -o|--output <path> (optional) \n"
                  <<            "\t\t <path>: Specifies the ouput directory path \n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help" <<std::endl;
        exit(EXIT_FAILURE);
    }
};



#endif

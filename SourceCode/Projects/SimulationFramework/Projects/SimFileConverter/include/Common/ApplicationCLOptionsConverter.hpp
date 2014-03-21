#ifndef ApplicationCLOptionsConverter_hpp
#define ApplicationCLOptionsConverter_hpp


#include <string>
#include <algorithm>
#include <unordered_map>

#include <boost/variant.hpp>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>

#include "AssertionDebug.hpp"
#include "Singleton.hpp"
#include "CommonFunctions.hpp"

#include "Exception.hpp"

#include "SimFileJoiner.hpp"

/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions: public Utilities::Singleton<ApplicationCLOptions> {
public:

    typedef SimFileJoiner::RangeAll RangeAll;
    typedef SimFileJoiner::TypesTimeRange TypesTimeRange;
    typedef SimFileJoiner::TypesBodyRange TypesBodyRange;

    std::vector<boost::filesystem::path> m_inputFiles;
    boost::filesystem::path m_outputFile;

    TypesTimeRange::VariantType m_timeRange;
    TypesBodyRange::VariantType m_bodyRange;

    enum class Task: unsigned int{
        UNDEFINED = 0,
        JOIN = 1
    };
    Task m_task;


    void parseOptions(int argc, char **argv) {

        m_timeRange = RangeAll();
        m_bodyRange = RangeAll();

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


                if( ops >> OptionPresent("timerange")) {
                    //parse in all times;
                    std::vector<double> range;
                    ops >> Option("timerange",range);
                    if(range.size()!=2 || (range[1]!=-1 && range[1]<0) || (range[0]!=-1 && range[0]<0) ||
                       (range[1] != -1  && range[1] < range[0])
                       ){
                        THROWEXCEPTION("Exception occured in parsing timerange: range.size()" )
                        printHelp();
                    }
                    m_timeRange = TypesTimeRange::RangeType(range[0],range[1]);

                }else if (ops >> OptionPresent("timelist")) {
                    std::vector<double> range;
                    ops >> Option("timelist",range);
                    if(range.size()==0){
                        THROWEXCEPTION("Exception occured in parsing timelist: range.size()" )
                        printHelp();
                    }
                    TypesTimeRange::ListType r(range.begin(), range.end());
                    m_timeRange = r;
                }



                 if( ops >> OptionPresent("bodyrange")) {
                    //parse in all times;
                    std::vector<unsigned int> range;
                    ops >> Option("bodyrange",range);
                    if(range.size()!=2){
                        THROWEXCEPTION("Exception occured in parsing bodyrange: range.size()" )
                        printHelp();
                    }
                    m_bodyRange = TypesBodyRange::RangeType(range[0],range[1]);

                }else if (ops >> OptionPresent("bodylist")) {
                    std::vector<unsigned int> range;
                    ops >> Option("bodylist",range);
                    if(range.size()==0){
                        THROWEXCEPTION("Exception occured in parsing bodylist: range.size()" )
                        printHelp();
                    }
                    TypesBodyRange::ListType l(range.begin(), range.end());
                    m_bodyRange = l;
                }



                ops >> Option('o',"output",m_outputFile);

            }else{
                THROWEXCEPTION("Exception occured in parsing task arg" )
                printHelp();
            }



        } catch(GetOpt::GetOptEx ex) {
            THROWEXCEPTION("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
            printHelp();
        }

        if (ops.options_remain()){
            THROWEXCEPTION("Some unexpected options where given!" )
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

            // Print ranges if possible;

        }else{
            s << "undefined";
        }
        s<<std::endl;
    }

    void checkArguments() {

        if(m_inputFiles.empty()) {
            THROWEXCEPTION( "No input files supplied!" )
            printHelp();
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {
                    THROWEXCEPTION( "Input file supplied as argument: " << *it << " does not exist!")
                    printHelp();
                }
            }

        }

        if(m_task == Task::JOIN){
            if(m_outputFile.empty()){
                THROWEXCEPTION( "No output file supplied!" )
                printHelp();
            }
            if(boost::filesystem::exists(m_outputFile)) {
                    THROWEXCEPTION( "Input file supplied as argument: " << m_outputFile << " does already exist (no overwrite is allowed)!")
                    printHelp();
            }
        }

    }

private:

    void printErrorNoArg(std::string arg) {
        THROWEXCEPTION( "Wrong options specified for arguement: '" << arg <<"'")
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

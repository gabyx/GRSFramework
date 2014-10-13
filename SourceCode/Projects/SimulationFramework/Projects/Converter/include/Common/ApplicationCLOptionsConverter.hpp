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
class ApplicationCLOptionsSimConverter: public Utilities::Singleton<ApplicationCLOptionsSimConverter>  {
public:

    using RangeAll = SimFileJoiner::RangeAll;
    using TypesTimeRange = SimFileJoiner::TypesTimeRange;
    using TypesBodyRange = SimFileJoiner::TypesBodyRange;

    std::vector<boost::filesystem::path> m_inputFiles;
    boost::filesystem::path m_outputFile;

    // Joiner
    TypesTimeRange::VariantType m_timeRange;
    TypesBodyRange::VariantType m_bodyRange;

    // Resampler
    unsigned int m_stepSize = 1;
    unsigned int m_startStateIdx = 0;
    unsigned int m_endStateIdx = std::numeric_limits<unsigned int>::max();

    enum class Task: unsigned int{
        JOIN = 1,
        RESAMPLE = 2
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
                exit(EXIT_SUCCESS);
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

                }
                else if (ops >> OptionPresent("timelist")) {
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
                    std::vector<long long int> range;
                    ops >> Option("bodyrange",range);
                    if(range.size()!=2 || (range[1]!=-1 && range[1]<0) || (range[0]!=-1 && range[0]<0) ||
                       (range[1] != -1  && range[1] < range[0]) ){
                        printHelp();
                        THROWEXCEPTION("Exception occured in parsing bodyrange: range.size()" )
                    }
                    m_bodyRange = TypesBodyRange::RangeType(range[0],range[1]);

                }else if (ops >> OptionPresent("bodylist")) {
                    std::vector<unsigned int> range;
                    ops >> Option("bodylist",range);
                    if(range.size()==0){
                        printHelp();
                        THROWEXCEPTION("Exception occured in parsing bodylist: range.size()" )

                    }
                    TypesBodyRange::ListType l(range.begin(), range.end());
                    m_bodyRange = l;
                }

            }else if(task == "resample"){

                m_task = Task::RESAMPLE;

                    //parse in start,step,end
                    ops >> Option("stepSize",m_stepSize);
                    m_stepSize = std::max(m_stepSize,1U);

                    if( ops >> OptionPresent("startIdx")) {
                        ops >> Option("startIdx",m_startStateIdx);
                    }
                    if( ops >> OptionPresent("endIdx")) {
                        ops >> Option("startIdx",m_endStateIdx);
                    }
                    if(m_endStateIdx<=m_startStateIdx){
                         THROWEXCEPTION("Exception occured: startIdx >= endIdx = " << m_endStateIdx )
                    }
            }else{
                printHelp();
                THROWEXCEPTION("Exception occured in parsing task arguments" )

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

            ops >> Option('o',"output",m_outputFile);
        }
        catch(GetOpt::ParsingErrorEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx ex) {
            printHelp();
            THROWEXCEPTION("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){
            printHelp();
            THROWEXCEPTION("Some unexpected options where given!" )
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
        }else if(m_task == Task::RESAMPLE){
            s << "resample";
        }else{
            s << "undefined";
        }
        s<<std::endl;
    }

    void checkArguments() {

        if(m_inputFiles.empty()) {
            printHelp();
            THROWEXCEPTION( "No input files supplied!" )
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {
                    printHelp();
                    THROWEXCEPTION( "Input file supplied as argument: " << *it << " does not exist!")
                }
            }

        }


        if(m_outputFile.empty()){
            printHelp();
            THROWEXCEPTION( "No output file supplied!" )
        }
        if(boost::filesystem::exists(m_outputFile)) {
                printHelp();
                THROWEXCEPTION( "Output file supplied as argument: " << m_outputFile << " does already exist (no overwrite is allowed)!")
        }


    }

private:

    void printErrorNoArg(std::string arg) {
        printHelp();
        THROWEXCEPTION( "Wrong options specified for arguement: '" << arg <<"'")
        exit(EXIT_FAILURE);
    }

    void printHelp() {
        std::cerr << "Help for the Application: \n Options: \n"
                  << " \t -i|--input <path1> <path2> ... \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path1> <path2> ... : These are multiple space delimited input sim file paths which are processed \n"
                  << " \t -t|--task join|resample \n"
                  << " \t [Required] \n"
                  <<            "\t\t This describes the task:\n"
                  <<            "\t\t\t 'join': Joins multiple sim files together into one file\n"
                  <<            "\t\t\t         Takes the following options:\n"
                  <<            "\t\t\t         --bodyrange <start> <end> | --bodylist <id1> <id2> ... \n"
                  <<            "\t\t\t         --timerange <start> <end> | --timelist <t1>  <t2> ... \n"
                  <<            "\t\t\t         Note: for option --bodyrange or --timerange: \n"
                  <<            "\t\t\t               if end=-1, then all times/bodies are taken! \n"
                  <<            "\t\t\t 'resample': Resample multiple sim files, each after the other\n"
                  <<            "\t\t\t         Takes the following options:\n"
                  <<            "\t\t\t         --step <number> \n"
                  <<            "\t\t\t         --start <stateIdx> --end <stateIdx> \n"
                  <<            "\t\t\t         Note: option --step needs to be greater than 1, start and end \n"
                  <<            "\t\t\t               represent state indices in the file.\n"
                  << " \t -o|--output <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the ouput file path \n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help\n";
    }
};





class ApplicationCLOptionsRenderer: public Utilities::Singleton<ApplicationCLOptionsRenderer> {
public:


    std::vector<boost::filesystem::path> m_inputFiles;
    boost::filesystem::path m_outputFile;

     // RenderConverter
    boost::filesystem::path m_sceneFile;
    boost::filesystem::path m_materialFile;

    enum class Renderer: unsigned int{
        RENDERMAN = 0,
        LUXRENDER = 1
    };
    Renderer m_renderer;


    void parseOptions(int argc, char **argv) {

        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);
        ops.exceptions_all();
        try {

            if( ops >> OptionPresent('h',"help")) {
                printHelp();
                exit(EXIT_SUCCESS);
            }

            std::string render;
            ops >> Option('r',"renderer",render);
            if(render == "renderman"){
                m_renderer = Renderer::RENDERMAN;
            }else if(render == "luxrender"){
                m_renderer = Renderer::LUXRENDER;
            }

            ops >> Option('s',"scene", m_sceneFile);

            ops >> Option('o',"output",m_outputFile);

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

        }
        catch(GetOpt::ParsingErrorEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx ex){
            printHelp();
            THROWEXCEPTION("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx ex) {
            printHelp();
            THROWEXCEPTION("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){
            printHelp();
            THROWEXCEPTION("Some unexpected options where given!" )
        }

    }

    void printArgs(std::ostream & s){
        s << " Input Files Arg: ";
        Utilities::printVector(s, m_inputFiles.begin(), m_inputFiles.end(), std::string(" , "));
        s <<std::endl;
        s << " Scene File Arg: " << m_sceneFile <<std::endl;
        s << " Output File Arg: " << m_outputFile <<std::endl;
        s << " Renderer: ";
        if(m_renderer == Renderer::RENDERMAN){
            s << "renderman";
        }else if(m_renderer == Renderer::LUXRENDER){
            s << "luxrender";
        }else{
            s << "undefined";
        }
        s<<std::endl;
    }

    void checkArguments() {

        if(m_inputFiles.empty()) {
            printHelp();
            THROWEXCEPTION( "No input files supplied!" )
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {
                    printHelp();
                    THROWEXCEPTION( "Input file supplied as argument: " << *it << " does not exist!")
                }
            }
        }

        if(m_sceneFile.empty()){
            printHelp();
            THROWEXCEPTION( "No scene file supplied!" )
        }else if(!boost::filesystem::exists(m_sceneFile)) {
            printHelp();
            THROWEXCEPTION( "Scene file supplied as argument: " << m_sceneFile << " does not exist!")
        }

        if(m_outputFile.empty()){
            printHelp();
            THROWEXCEPTION( "No output file supplied!" )
        }else if(boost::filesystem::exists(m_outputFile)) {
                printHelp();
                THROWEXCEPTION( "Output file supplied as argument: " << m_outputFile << " does already exist (no overwrite is allowed)!")
        }


    }

private:

    void printErrorNoArg(std::string arg) {
        printHelp();
        THROWEXCEPTION( "Wrong options specified for arguement: '" << arg <<"'")
        exit(EXIT_FAILURE);
    }

    void printHelp() {
        std::cerr << "Help for the Application: \n Options: \n"
                  << " \t -i|--input <path1> <path2> ... \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path1> <path2> ... : These are multiple space delimited input sim file paths which are processed \n"
                  << " \t -r|--renderer renderman|luxrender \n"
                  << " \t [Required] \n"
                  << " \t -s|--scene <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the scene file xml path \n"
                  << " \t -o|--output <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the ouput directory path \n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help\n";
    }
};




#endif

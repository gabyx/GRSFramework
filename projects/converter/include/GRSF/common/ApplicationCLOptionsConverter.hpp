// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_ApplicationCLOptionsConverter_hpp
#define GRSF_common_ApplicationCLOptionsConverter_hpp


#include <string>
#include <algorithm>
#include <unordered_map>

#include <boost/variant.hpp>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/Singleton.hpp"
#include "GRSF/common/CommonFunctions.hpp"

#include "GRSF/common/Exception.hpp"

#include "GRSF/converters/simJoiner/SimFileJoiner.hpp"

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
    unsigned int m_increment = 1;
    unsigned int m_startStateIdx = 0;
    unsigned int m_endStateIdx = std::numeric_limits<unsigned int>::max();
    bool m_splitIntoFiles = false;

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
                        ERRORMSG("Exception occured in parsing timerange: range.size()" )
                    }
                    m_timeRange = TypesTimeRange::RangeType(range[0],range[1]);

                }
                else if (ops >> OptionPresent("timelist")) {
                    std::vector<double> range;
                    ops >> Option("timelist",range);
                    if(range.size()==0){
                        ERRORMSG("Exception occured in parsing timelist: range.size()" )
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
                        ERRORMSG("Exception occured in parsing bodyrange: range.size()" )
                    }
                    m_bodyRange = TypesBodyRange::RangeType(range[0],range[1]);

                }else if (ops >> OptionPresent("bodylist")) {
                    std::vector<unsigned int> range;
                    ops >> Option("bodylist",range);
                    if(range.size()==0){
                        ERRORMSG("Exception occured in parsing bodylist: range.size()" )

                    }
                    TypesBodyRange::ListType l(range.begin(), range.end());
                    m_bodyRange = l;
                }

            }else if(task == "resample"){

                m_task = Task::RESAMPLE;

                    //parse in start,step,end
                    ops >> Option("increment",m_increment);
                    m_increment = std::max(m_increment,1U);

                    if( ops >> OptionPresent("startIdx")) {
                        ops >> Option("startIdx",m_startStateIdx);
                    }
                    if( ops >> OptionPresent("endIdx")) {
                        ops >> Option("endIdx",m_endStateIdx);
                    }
                    if( ops >> OptionPresent("split")) {
                        m_splitIntoFiles = true;
                    }
                    if(m_endStateIdx<m_startStateIdx){
                         ERRORMSG("Exception occured: startIdx >= endIdx = " << m_endStateIdx )
                    }
            }else{
                ERRORMSG("Exception occured in parsing task arguments" )

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
        catch(GetOpt::ParsingErrorEx & ex){
            ERRORMSG("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx & ex){
            ERRORMSG("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx & ex){
            ERRORMSG("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx & ex){
            ERRORMSG("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx & ex){
            ERRORMSG("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx ex){
            ERRORMSG("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx & ex) {
            ERRORMSG("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){
            ERRORMSG("Some unexpected options where given!" )
        }

    }

    void printArgs(std::ostream & s){
        s << "---> Input Files Arg: \n\t";
        Utilities::printVector(s, m_inputFiles.begin(), m_inputFiles.end(), std::string(" , "));
        s << std::endl;
        s << "\t Output File Arg: " << m_outputFile <<std::endl;
        s << "\t Task Arg: ";
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
            ERRORMSG( "No input files supplied!" )
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {
                    ERRORMSG( "Input file supplied as argument: " << *it << " does not exist!")
                }
            }

        }


        if(m_outputFile.empty()){
            ERRORMSG( "No output file supplied!" )
        }
        if(boost::filesystem::exists(m_outputFile)) {
                ERRORMSG( "Output file supplied as argument: " << m_outputFile << " does already exist (no overwrite is allowed)!")
        }


    }

private:

    void printErrorNoArg(std::string arg) {
        ERRORMSG( "Wrong options specified for arguement: '" << arg <<"'")
        exit(EXIT_FAILURE);
    }

    void printHelp() {
        std::cerr << "Help for the Application Sim: \n Options: \n"
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
                  <<            "\t\t\t         --increment <number> \n"
                  <<            "\t\t\t         --startIdx <number> --endIdx <number> \n"
                  <<            "\t\t\t         --split \n"
                  <<            "\t\t\t         Note: option --step needs to be greater than 1, start and end \n"
                  <<            "\t\t\t               represent state indices in the file.\n"
                  <<            "\t\t\t               --split splits all states into sucessive files.\n"
                  << " \t -o|--output <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the ouput file path \n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help\n";
    }
};


/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptionsSimInfo: public Utilities::Singleton<ApplicationCLOptionsSimInfo>  {
public:

    bool m_prettyPrint = false;
    bool m_noTimeList  = false;
    bool m_skipFirstState = false;
    unsigned int m_increment = 1;
    unsigned int m_startStateIdx = 0;
    unsigned int m_endStateIdx = std::numeric_limits<unsigned int>::max();

    std::vector<boost::filesystem::path> m_inputFiles;

    void parseOptions(int argc, char **argv) {


        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);
        ops.exceptions_all();
        try {

            if( ops >> OptionPresent('h',"help")) {
                printHelp();
                exit(EXIT_SUCCESS);
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

            if( ops >> OptionPresent("increment") ){
                ops >> Option("increment",m_increment);
                m_increment = std::max(m_increment,1U);
            }

            if( ops >> OptionPresent("startIdx")) {
                ops >> Option("startIdx",m_startStateIdx);
            }
            if( ops >> OptionPresent("endIdx")) {
                ops >> Option("endIdx",m_endStateIdx);
            }
            if(m_endStateIdx<m_startStateIdx){
                 ERRORMSG("Exception occured: startIdx >= endIdx = " << m_endStateIdx )
            }

            if( ops >> OptionPresent("skipFirstState")) {
                m_skipFirstState = true;
            }

            if( ops >> OptionPresent('x',"prettyPrint")) {
                m_prettyPrint = true;
            }

            if( ops >> OptionPresent('t',"noTimeList")) {
                m_noTimeList = true;
            }


        }
        catch(GetOpt::ParsingErrorEx & ex){

            ERRORMSG("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx & ex){

            ERRORMSG("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx & ex){

            ERRORMSG("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx & ex){

            ERRORMSG("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx & ex){

            ERRORMSG("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx ex){

            ERRORMSG("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx & ex) {

            ERRORMSG("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){

            ERRORMSG("Some unexpected options where given!" )
        }

    }

    void printArgs(std::ostream & s){
        s << "---> Input Files Arg: \n\t";
        Utilities::printVector(s, m_inputFiles.begin(), m_inputFiles.end(), std::string(" , "));
        s<<std::endl;
    }

    void checkArguments() {

        if(m_inputFiles.empty()) {

            ERRORMSG( "No input files supplied!" )
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {

                    ERRORMSG( "Input file supplied as argument: " << *it << " does not exist!")
                }
            }

        }
    }

private:

    void printErrorNoArg(std::string arg) {

        ERRORMSG( "Wrong options specified for arguement: '" << arg <<"'")
        exit(EXIT_FAILURE);
    }

    void printHelp() {
        std::cerr << "Help for the Application SimInfo: \n Options: \n"
                  << "\t -i|--input <path1> <path2> ... \n"
                  <<    "\t\t [Required] \n"
                  <<    "\t\t <path1> <path2> ... : These are multiple space delimited input sim file paths which are processed \n"
                  << "\t --increment <number> \n"
                  << "\t --startIdx <number> \n"
                  << "\t --endIdx <number> \n"
                  << "\t -s|--skipFirstState \n"
                  <<    "\t\t Note: These --startIdx, --endIdx, --increment can be used\n"
                  <<    "\t\t       to track the resample ranges over multiple sim files.\n"
                  <<    "\t\t       --increment needs to be greater than 1, start and end \n"
                  <<    "\t\t       represent state indices in the file.\n"
                  <<    "\t\t       --skipFirstState skips all first states after the first\n"
                  <<    "\t\t       file. Indices startIdx, endIdx are counted with respect\n"
                  <<    "\t\t       to this flag\n"
                  << "\t -x|--prettyPrint \n"
                  <<    "\t\t Outputs nice formatted strings rather than XML \n"
                  << "\t -t|--noTimeList \n"
                  <<    "\t\t Outputs no time information \n";
    }
};






class ApplicationCLOptionsRenderer: public Utilities::Singleton<ApplicationCLOptionsRenderer> {
public:
    enum class Renderer: unsigned int{
        RENDERMAN = 0,
        LUXRENDER = 1
    };
private:


    std::vector<boost::filesystem::path> m_inputFiles;

    boost::filesystem::path m_outputFile;

     // RenderConverter
    boost::filesystem::path m_sceneFile;
    boost::filesystem::path m_mediaDir ="./";
    boost::filesystem::path m_converterLogicFile;


    Renderer m_renderer;

public:


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
                ERRORMSG("Luxrender is not supported at the moment!")
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

            ops >> Option('s',"scene", m_sceneFile);


            if( ops >> OptionPresent('m',"media-path")) {
                ops >> Option('m',"media-path",m_mediaDir);
            }

            ops >> Option('c',"converterLogic", m_converterLogicFile);

            if(ops >> OptionPresent('o',"outputFile")){
                ops >> Option('o',"outputFile",m_outputFile);
            }

        }
        catch(GetOpt::ParsingErrorEx & ex){

            ERRORMSG("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx & ex){

            ERRORMSG("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx & ex){

            ERRORMSG("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx & ex){

            ERRORMSG("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx & ex){

            ERRORMSG("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx & ex){

            ERRORMSG("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx & ex) {

            ERRORMSG("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){

            ERRORMSG("Some unexpected options where given!" )
        }

    }

    void printArgs(std::ostream & s){
        s << "---> Input Files Arg: ";
        Utilities::printVector(s, m_inputFiles.begin(), m_inputFiles.end(), std::string(" , "));
        s <<std::endl;
        s << "\tScene File Arg: " << m_sceneFile <<std::endl;
        s << "\tOutput File Arg: " << m_outputFile <<std::endl;
        s << "\tRenderer: ";
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

            ERRORMSG( "No input files supplied!" )
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {

                    ERRORMSG( "Input file supplied as argument: " << *it << " does not exist!")
                }
            }
        }

        if(m_sceneFile.empty()){

            ERRORMSG( "No scene file supplied!" )
        }else if(!boost::filesystem::exists(m_sceneFile)) {

            ERRORMSG( "Scene file supplied as argument: " << m_sceneFile << " does not exist!")
        }

        if(!boost::filesystem::exists(m_mediaDir)) {
            ERRORMSG( "Media directory " << m_mediaDir << " does not exist!" )
        }
    }

    inline const std::vector<boost::filesystem::path> & getInputFiles(){ return m_inputFiles;}
    inline const boost::filesystem::path & getMediaDir(){ return m_mediaDir;}
    inline const boost::filesystem::path & getOutputFile(){ return m_outputFile;}
    inline const boost::filesystem::path & getConverterLogicFile(){ return m_converterLogicFile;}
    inline const boost::filesystem::path & getSceneFile(){ return m_sceneFile;}
    inline Renderer getRenderer(){ return m_renderer;}

private:

    void printErrorNoArg(std::string arg) {

        ERRORMSG( "Wrong options specified for argument: '" << arg <<"'")
    }

    void printHelp() {
        std::cerr << "Help for the Application Renderer: \n Options: \n"
                  << " \t -i|--input <path1> <path2> ... \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path1> <path2> ... : These are multiple space delimited input sim (.sim)\n"
                  <<            "\t\t file paths which are processed. \n"
                  << " \t -r|--renderer renderman|luxrender \n"
                  << " \t [Required] \n"
                  << " \t\t luxrender output is not supported yet!, renderman means any renderman interface byte\n"
                  << " \t\t stream (RIB) compliant renderer."
                  << " \t -s|--scene <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the scene file xml path to use for converting. \n"
                  << " \t -c|--converterLogic <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the converter logic file xml path. \n"
                  << " \t -m|--media-path <path> \n"
                  << " \t [Optional] \n"
                  <<            "\t\t <path>: is the base directory for all media files (.obj, .mesh) \n"
                  <<            "\t\t which is used for relative file names in the scene file <SceneFilePath>. (no slash at the end)\n"
                  <<            "\t\t if not specified the media directory is './' .\n"
                  << " \t -o|--outputFile <path>  \n"
                  << " \t [Optional] \n"
                  <<            "\t\t <path>: Specifies the output base file path used for each state.\n"
                  <<            "\t\t Example: -o dir1/dir2/outFrame -> [outputDir= 'dir1/dir2/', fileName = 'outFrame' ]\n"
                  <<            "\t\t Example: -o dir1/dir2/ -> [outputDir= 'dir1/dir2/', fileName = 'Frame' ].\n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help.\n";
    }
};



class ApplicationCLOptionsAnalyzer: public Utilities::Singleton<ApplicationCLOptionsAnalyzer> {
public:

private:

    std::vector<boost::filesystem::path> m_inputFiles;

    boost::filesystem::path m_outputFile;

    boost::filesystem::path m_sceneFile;
    boost::filesystem::path m_mediaDir ="./";
    boost::filesystem::path m_converterLogicFile;
protected:
    std::string m_name = "Analyzer";

public:

    void parseOptions(int argc, char **argv) {

        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);
        ops.exceptions_all();
        try {

            if( ops >> OptionPresent('h',"help")) {
                printHelp();
                exit(EXIT_SUCCESS);
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

            if( ops >> OptionPresent('s',"scene")) {
                ops >> Option('s',"scene", m_sceneFile);
            }

            if( ops >> OptionPresent('m',"media-path")) {
                ops >> Option('m',"media-path",m_mediaDir);
            }

            ops >> Option('c',"converterLogic", m_converterLogicFile);

            if(ops >> OptionPresent('o',"outputFile")){
                ops >> Option('o',"outputFile",m_outputFile);
            }
        }
        catch(GetOpt::ParsingErrorEx & ex){

            ERRORMSG("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx & ex){

            ERRORMSG("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx & ex){

            ERRORMSG("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx & ex){

            ERRORMSG("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx & ex){

            ERRORMSG("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx & ex){

            ERRORMSG("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx & ex) {

            ERRORMSG("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){

            ERRORMSG("Some unexpected options where given!" )
        }

    }

    void printArgs(std::ostream & s){
        s << "---> Input Files Arg: ";
        Utilities::printVector(s, m_inputFiles.begin(), m_inputFiles.end(), std::string(" , "));
        s <<std::endl;
        s << "\tScene File Arg: " << m_sceneFile <<std::endl;
        s << "\tOutput File Arg: " << m_outputFile <<std::endl;
        s << "\tRenderer: ";
        s<<std::endl;
    }

    void checkArguments() {

        if(m_inputFiles.empty()) {
            ERRORMSG( "No input files supplied!" )
        } else {
            for(auto it = m_inputFiles.begin(); it != m_inputFiles.end(); it++){
                if(! boost::filesystem::exists(*it)) {
                    ERRORMSG( "Input file supplied as argument: " << *it << " does not exist!")
                }
            }
        }

        if(!m_sceneFile.empty()){
            if(!boost::filesystem::exists(m_sceneFile)) {
                ERRORMSG( "Scene file supplied as argument: " << m_sceneFile << " does not exist!")
                }
        }

        if(!m_mediaDir.empty()){
            if(!boost::filesystem::exists(m_mediaDir)) {
                ERRORMSG( "Media directory " << m_mediaDir << " does not exist!" )
            }
        }
    }

    inline const std::vector<boost::filesystem::path> & getInputFiles(){ return m_inputFiles;}
    inline const boost::filesystem::path & getMediaDir(){ return m_mediaDir;}
    inline const boost::filesystem::path & getOutputFile(){ return m_outputFile;}
    inline const boost::filesystem::path & getConverterLogicFile(){ return m_converterLogicFile;}
    inline const boost::filesystem::path & getSceneFile(){ return m_sceneFile;}

private:

    void printErrorNoArg(std::string arg) {

        ERRORMSG( "Wrong options specified for arguement: '" << arg <<"'")
    }

    void printHelp() {
        std::cerr << "Help for the Application "<<m_name<<": \n Options: \n"
                  << " \t -i|--input <path1> <path2> ... \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path1> <path2> ... : These are multiple space delimited input sim (.sim)\n"
                  <<            "\t\t file paths which are processed. \n"
                  << " \t -s|--scene <path>  \n"
                  << " \t [Optional] \n"
                  <<            "\t\t <path>: Specifies the scene file xml path to use for converting. \n"
                  << " \t -c|--converterLogic <path>  \n"
                  << " \t [Required] \n"
                  <<            "\t\t <path>: Specifies the converter logic file xml path. \n"
                  << " \t -m|--media-path <path> \n"
                  << " \t [Optional] \n"
                  <<            "\t\t <path>: is the base directory for all media files (.obj, .mesh) \n"
                  <<            "\t\t which is used for relative file names in the scene file <SceneFilePath>. (no slash at the end)\n"
                  <<            "\t\t if not specified the media directory is './' .\n"
                  << " \t -o|--outputFile <path>  \n"
                  << " \t [Optional] \n"
                  <<            "\t\t <path>: Specifies the ouput base file path used for each state.\n"
                  <<            "\t\t For relative file paths , the --outputDir file paths is appended if given.\n"
                  <<            "\t\t Example: -o dir1/dir2/outFrame is used to initialized the FrameData source node.\n"
                  <<            "\t\t since it is not a absolute path, the --outputDir path is appenden if given!.\n"
                  << " \t -o|--outputFile <path>  \n"
                  << " \t [Optional] \n"
                  <<            "\t\t <path>: Specifies the output base file path used for each state.\n"
                  <<            "\t\t Example: -o dir1/dir2/outFrame -> [outputDir= 'dir1/dir2/', fileName = 'outFrame' ]\n"
                  <<            "\t\t Example: -o dir1/dir2/ -> [outputDir= 'dir1/dir2/', fileName = 'Frame' ].\n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help.\n";
    }
};


class ApplicationCLOptionsGridder: public ApplicationCLOptionsAnalyzer{
 public:
     ApplicationCLOptionsGridder(){m_name = "Gridder";}
};

#endif

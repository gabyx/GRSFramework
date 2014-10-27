#ifndef ApplicationCLOptions_hpp
#define ApplicationCLOptions_hpp


#include <string>
#include <algorithm>
#include <unordered_map>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>


#include "Singleton.hpp"
#include "CommonFunctions.hpp"



/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions: public Utilities::Singleton<ApplicationCLOptions> {
public:


    class PostProcessTask{
        public:
        PostProcessTask(const std::string & name):m_name(name){};

        std::string getName(){return m_name;}

        virtual void addOption(unsigned int index, const std::string &option){
            m_options[index] = option;
        }

        virtual void execute() = 0;

        virtual ~PostProcessTask(){};

        protected:

            friend std::ostream & operator<<(std::ostream & s, const PostProcessTask & p);

            std::string m_name;
            std::unordered_map<unsigned int, std::string> m_options;
    };

    class PostProcessTaskBash : public PostProcessTask{
        public:
            PostProcessTaskBash(const std::string & name): PostProcessTask(name){}
            void execute(){

                system( this->m_options[1].c_str());
            }
    };

    class PostProcessTaskCopyLocalTo : public PostProcessTask{
        public:
            PostProcessTaskCopyLocalTo(const std::string & name): PostProcessTask(name){}
            void addOption(unsigned int index, const std::string &option){

            }
            virtual void execute(){

            }
    };


    std::vector<boost::filesystem::path> m_localDirs;
    boost::filesystem::path m_globalDir = "./";
    boost::filesystem::path m_sceneFile;

    // If I would like to have some posst process tasks going on afterwards
    // Like bash command and so one
    // This is not needed sofar in MPI mode, because I can do this with mpirun -npernode 1 command and so on which lets me postprocess files
    // per node on the cluster or
    std::vector<PostProcessTask *> m_postProcessTasks;

    ~ApplicationCLOptions(){
        for(auto it = m_postProcessTasks.begin(); it != m_postProcessTasks.end(); ++it){
            delete (*it);
        }
    }


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
                for(auto it = svec.begin(); it != svec.end(); ++it){
                    m_localDirs.push_back(*it);
                }
            }else{
                m_localDirs.push_back(m_globalDir);
            }

            if( ops >> OptionPresent('p', "post-process")) {
                std::vector<std::string> svec;
                ops >> Option('p',"post-process",svec);

                unsigned int currentArgIdx = 0;
                unsigned int nextArgIdx = 0;
                PostProcessTask * p = nullptr;
                for(unsigned int i = 0; i < svec.size(); i++){
                    if(svec[i] == "bash"){
                        if(i != nextArgIdx){
                            printHelp();
                           ERRORMSG( "Postprocess Argument: " << "bash" << " at wrong position!" << std::endl);
                        }

                        // has 2 arguments [int|all] and string which is the bash command!
                       currentArgIdx = i;
                       nextArgIdx = i + 3;
                       if(nextArgIdx-1 >=  svec.size()){
                            printHelp();
                            ERRORMSG("Postprocess Argument: " << "bash" << ", two little arguments!" << std::endl);

                       }
                       m_postProcessTasks.push_back(new PostProcessTaskBash("bash"));
                       p = m_postProcessTasks.back();
                    }else if( svec[i] == "copy-local-to"){
                        if(i != nextArgIdx){
                            printHelp();
                            ERRORMSG("Postprocess Argument: " << "copy-local-to" << " at wrong position!" << std::endl)
                        }

                        currentArgIdx = i;
                        nextArgIdx = i + 1;
                        if(nextArgIdx-1 >=  svec.size()){
                            printHelp();
                            ERRORMSG("Postprocess Argument: " << "copy-local-to" << ", two little arguments!" << std::endl);
                        }
                        m_postProcessTasks.push_back(new PostProcessTaskCopyLocalTo("copy-local-to"));
                        p = m_postProcessTasks.back();
                    }else{
                        if(i >= nextArgIdx){
                            printHelp();
                            ERRORMSG("Postprocess Argument: " << svec[i] << " not known!" << std::endl);
                        }
                        if(p){
                            p->addOption(i-currentArgIdx-1,svec[i]);
                        }
                        // otherwise skip (it belongs to a argument befor
                    }
                }




            }

        }
        catch(GetOpt::ParsingErrorEx & ex){
            printHelp();
            ERRORMSG("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::InvalidFormatEx & ex){
            printHelp();
            ERRORMSG("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionNotFoundEx & ex){
            printHelp();
            ERRORMSG("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyArgumentsEx & ex){
            printHelp();
            ERRORMSG("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::TooManyOptionsEx & ex){
            printHelp();
            ERRORMSG("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what() )
        }
        catch(GetOpt::OptionsFileNotFoundEx & ex){
            printHelp();
            ERRORMSG("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what() )
        } catch(GetOpt::GetOptEx & ex) {
            printHelp();
            ERRORMSG("GetOpt::GetOptEx exception occured in parsing args: " << ex.what() )
        }

        if (ops.options_remain()){
            printHelp();
            ERRORMSG("Some unexpected options where given!" << std::endl)
        }


        if(m_localDirs.size()==0) {
            m_localDirs.push_back("./");
        }


    }

    void printArgs(std::ostream & s){
        s << " SceneFile Arg: " << m_sceneFile <<std::endl;
        s << " GlobalFilePath Arg: " << m_globalDir <<std::endl;
        s << " LocalFilePaths Args: ";
        Utilities::printVector(s, m_localDirs.begin(), m_localDirs.end(), std::string(" "));
        s << std::endl;

        for(auto it = m_postProcessTasks.begin(); it != m_postProcessTasks.end(); ++it){
            s << *(*it);
        }
        //exit(-1);
    }

    void checkArguments() {
        if(m_sceneFile.empty()) {
            ERRORMSG("No scene file (.xml) supplied as argument: -s [SceneFilePath]" << std::endl)
            printHelp();
        } else {
            if(! boost::filesystem::exists(m_sceneFile)) {
                printHelp();
                ERRORMSG("Scene file supplied as argument: " << m_sceneFile << " does not exist!"<< std::endl)
            }
        }
    }

private:

    void printErrorNoArg(std::string arg) {
        printHelp();
        ERRORMSG("Wrong options specified for arguement: '" << arg <<"'"<< std::endl)
    }

    void printHelp() {
        std::cerr << "Help for the Application:" << std::endl <<"Options:" <<std::endl
                  << " \t -s <SceneFilePath> \n"
                  <<            "\t\t <SceneFilePath>: is a .xml file path for the scene, essential \n"
                  <<            "\t\t for CLI version, in GUI version: \"SceneFile.xml\" is the default file \n"
                  << " \t -g|--global-path <GlobalDirectoryPath> (optional) \n"
                  <<            "\t\t <GlobalDirectoryPath>: is the global directory path. (no slash at the end, boost::create_directory bug!)\n"
                  << " \t -l|--local-path <LocalDirectoryPath> (optional) \n"
                  <<            "\t\t <LocalDirectoryPath>: is the local directory for each processes output, \n"
                  <<            "\t\t if not specified the local directory is the same as the global directory.\n"
                  <<            "\t\t (no slash at the end, boost::create_directory bug!)\n"
                  <<            "\t\t This can also be a list of directories (space delimited), which is \n"
                  <<            "\t\t distributed linearly over all participating processes.\n"
                  << " \t -p|--post-process bash|copy-local-to (optional) \n"
                  <<            "\t\t Various post process task which can be launched ath the end of the simulation:\n"
                  <<            "\t\t bash <int>|all <command> \n"
                  <<            "\t\t\t <command>: specifies a quoted bash command as \"rm -r ./dir/\" \n"
                  <<            "\t\t\t <int> is the rank number on which the bash command is launched, \n"
                  <<            "\t\t\t 'all' specifies all process who execute this command \n"
                  <<            "\t\t copy-local-to <path> \n"
                  <<            "\t\t\t Copies the local directory which is process specific to another folder at <path> \n"
                  << " \t -h|--help \n"
                  <<            "\t\t Prints this help" <<std::endl;
    }
};

// Global implementation for ApplicationCLOptions::PostProcessTask
std::ostream & operator<<(std::ostream & s, const ApplicationCLOptions::PostProcessTask & p);



#endif

// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_ApplicationCLOptions_hpp
#define GRSF_common_ApplicationCLOptions_hpp

#include <algorithm>
#include <boost/filesystem.hpp>
#include <getoptpp/getopt_pp_standalone.h>
#include <string>
#include <unordered_map>

#include "GRSF/common/CommonFunctions.hpp"
#include "GRSF/common/Singleton.hpp"

/**
*  @brief CommandLineOptions for the Application
*/
class ApplicationCLOptions : public Utilities::Singleton<ApplicationCLOptions>
{
private:
    std::vector<boost::filesystem::path> m_localDirs;
    boost::filesystem::path m_globalDir = "./";
    boost::filesystem::path m_mediaDir  = "./";
    boost::filesystem::path m_sceneFile;

public:
    class PostProcessTask
    {
    public:
        PostProcessTask(const std::string& name) : m_name(name){};

        std::string getName() const
        {
            return m_name;
        }

        virtual void addOption(unsigned int index, const std::string& option)
        {
            m_options[index] = option;
        }

        virtual int execute() const = 0;

        virtual ~PostProcessTask(){};

    protected:
        friend std::ostream& operator<<(std::ostream& s, const PostProcessTask& p);

        std::string m_name;
        std::unordered_map<unsigned int, std::string> m_options;
    };

    class PostProcessTaskBash : public PostProcessTask
    {
    public:
        PostProcessTaskBash(const std::string& name) : PostProcessTask(name)
        {
        }
        int execute() const
        {
            return system(this->m_options.at(1).c_str());
        }
    };

    class PostProcessTaskCopyLocalTo : public PostProcessTask
    {
    public:
        PostProcessTaskCopyLocalTo(const std::string& name) : PostProcessTask(name)
        {
        }
        void addOption(unsigned int index, const std::string& option)
        {
        }
        virtual int execute() const
        {
            return 1;
        }
    };

    ~ApplicationCLOptions()
    {
        for (auto it = m_postProcessTasks.begin(); it != m_postProcessTasks.end(); ++it)
        {
            delete (*it);
        }
    }

    inline const std::vector<boost::filesystem::path>& getLocalDirs()
    {
        return m_localDirs;
    }
    inline const boost::filesystem::path& getGlobalDir()
    {
        return m_globalDir;
    }
    inline const boost::filesystem::path& getMediaDir()
    {
        return m_mediaDir;
    }
    inline const boost::filesystem::path& getSceneFile()
    {
        return m_sceneFile;
    }

    inline const std::vector<const PostProcessTask*>& getPostProcessTasks()
    {
        return m_postProcessTasks;
    }

    void parseOptions(int argc, char** argv)
    {
        using namespace GetOpt;
        GetOpt::GetOpt_pp ops(argc, argv);
        ops.exceptions_all();
        try
        {
            if (ops >> OptionPresent('h', "help"))
            {
                printHelp();
                exit(EXIT_SUCCESS);
            }

            std::string s;
            ops >> Option('s', s);
            m_sceneFile = boost::filesystem::path(s);

            if (ops >> OptionPresent('g', "global-path"))
            {
                ops >> Option('g', "global-path", s);
                m_globalDir = boost::filesystem::path(s);
            }

            if (ops >> OptionPresent('l', "local-path"))
            {
                std::vector<std::string> svec;
                ops >> Option('l', "local-path", svec);
                for (auto it = svec.begin(); it != svec.end(); ++it)
                {
                    m_localDirs.push_back(*it);
                }
            }
            else
            {
                m_localDirs.push_back(m_globalDir);
            }

            if (ops >> OptionPresent('m', "media-path"))
            {
                ops >> Option('m', "media-path", s);
                m_mediaDir = boost::filesystem::path(s);
            }

            if (ops >> OptionPresent('p', "post-process"))
            {
                std::vector<std::string> svec;
                ops >> Option('p', "post-process", svec);

                unsigned int currentArgIdx = 0;
                unsigned int nextArgIdx    = 0;
                PostProcessTask* p         = nullptr;
                for (unsigned int i = 0; i < svec.size(); i++)
                {
                    if (svec[i] == "bash")
                    {
                        if (i != nextArgIdx)
                        {
                            GRSF_ERRORMSG("Postprocess Argument: "
                                          << "bash"
                                          << " at wrong position!"
                                          << std::endl);
                        }

                        // has 2 arguments [int|all] and string which is the bash command!
                        currentArgIdx = i;
                        nextArgIdx    = i + 3;
                        if (nextArgIdx - 1 >= svec.size())
                        {
                            GRSF_ERRORMSG("Postprocess Argument: "
                                          << "bash"
                                          << ", two little arguments!"
                                          << std::endl);
                        }
                        p = new PostProcessTaskBash("bash");
                        m_postProcessTasks.push_back(p);
                    }
                    else if (svec[i] == "copy-local-to")
                    {
                        if (i != nextArgIdx)
                        {
                            GRSF_ERRORMSG("Postprocess Argument: "
                                          << "copy-local-to"
                                          << " at wrong position!"
                                          << std::endl)
                        }

                        currentArgIdx = i;
                        nextArgIdx    = i + 1;
                        if (nextArgIdx - 1 >= svec.size())
                        {
                            GRSF_ERRORMSG("Postprocess Argument: "
                                          << "copy-local-to"
                                          << ", two little arguments!"
                                          << std::endl);
                        }
                        p = new PostProcessTaskCopyLocalTo("copy-local-to");
                        m_postProcessTasks.push_back(p);
                    }
                    else
                    {
                        if (i >= nextArgIdx)
                        {
                            GRSF_ERRORMSG("Postprocess Argument: " << svec[i] << " not known!" << std::endl);
                        }
                        if (p)
                        {
                            p->addOption(i - currentArgIdx - 1, svec[i]);
                        }
                        // otherwise skip (it belongs to a argument befor
                    }
                }
            }
        }
        catch (GetOpt::ParsingErrorEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::ParsingErrorEx exception occured in parsing args: " << ex.what())
        }
        catch (GetOpt::InvalidFormatEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::InvalidFormatEx exception occured in parsing args: " << ex.what())
        }
        catch (GetOpt::OptionNotFoundEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::OptionNotFoundEx exception occured in parsing args: " << ex.what())
        }
        catch (GetOpt::TooManyArgumentsEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::TooManyArgumentsEx exception occured in parsing args: " << ex.what())
        }
        catch (GetOpt::TooManyOptionsEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::TooManyOptionsEx exception occured in parsing args: " << ex.what())
        }
        catch (GetOpt::OptionsFileNotFoundEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::OptionsFileNotFoundEx exception occured in parsing args: " << ex.what())
        }
        catch (GetOpt::GetOptEx& ex)
        {
            GRSF_ERRORMSG("GetOpt::GetOptEx exception occured in parsing args: " << ex.what())
        }

        if (ops.options_remain())
        {
            GRSF_ERRORMSG("Some unexpected options where given!" << std::endl)
        }

        if (m_localDirs.size() == 0)
        {
            m_localDirs.push_back("./");
        }
    }

    void printArgs(std::ostream& s)
    {
        s << "---> Program Arguments::" << std::endl;
        s << "     SceneFile: \t\t" << m_sceneFile << std::endl;
        s << "     GlobalFilePath: \t\t" << m_globalDir << std::endl;
        s << "     LocalFilePaths: \t\t";
        Utilities::printVector(s, m_localDirs.begin(), m_localDirs.end(), std::string(" "));
        s << std::endl;
        s << "     MediaFilePath: \t\t" << m_mediaDir << std::endl;

        for (auto it = m_postProcessTasks.begin(); it != m_postProcessTasks.end(); ++it)
        {
            s << *(*it);
        }
        // exit(-1);
    }

    void checkArguments()
    {
        if (m_sceneFile.empty())
        {
            GRSF_ERRORMSG("No scene file (.xml) supplied as argument: -s [SceneFilePath]" << std::endl)
        }
        else
        {
            if (!boost::filesystem::exists(m_sceneFile))
            {
                GRSF_ERRORMSG("Scene file supplied as argument: " << m_sceneFile << " does not exist!" << std::endl)
            }
        }

        if (!boost::filesystem::exists(m_mediaDir))
        {
            GRSF_ERRORMSG("Media directory supplied as argument: " << m_mediaDir << " does not exist!" << std::endl)
        }
    }

private:
    // If I would like to have some posst process tasks going on afterwards
    // Like bash command and so one
    // This is not needed sofar in MPI mode, because I can do this with mpirun -npernode 1 command and so on which lets
    // me postprocess files
    // per node on the cluster or
    std::vector<const PostProcessTask*> m_postProcessTasks;

    void printErrorNoArg(std::string arg)
    {
        GRSF_ERRORMSG("Wrong options specified for arguement: '" << arg << "'" << std::endl)
    }

    void printHelp()
    {
        std::cerr
            << "Help for the Application:" << std::endl
            << "Options:" << std::endl
            << " \t -s <path> \n"
            << "\t\t <path>: is a .xml file path for the scene, essential \n"
            << "\t\t for CLI version, in GUI version: \"SceneFile.xml\" is the default file \n"
            << " \t -g|--global-path <path> (optional) \n"
            << "\t\t <path>: is the global directory path. (no slash at the end)\n"
            << "\t\t if not specified the media directory is './' .\n"
            << " \t -l|--local-path <path> (optional) \n"
            << "\t\t <path>: is the local directory for each processes output, \n"
            << "\t\t if not specified the local directory is the same as the global directory.\n"
            << "\t\t (no slash at the end, boost::create_directory bug!)\n"
            << "\t\t This can also be a list of directories (space delimited), which is \n"
            << "\t\t distributed linearly over all participating processes.\n"
            << " \t -m|--media-path <path> (optional) \n"
            << "\t\t <path>: is the base directory for all media files (.obj, .mesh) \n"
            << "\t\t which is used for relative file names in the scene file <SceneFilePath>. (no slash at the end)\n"
            << "\t\t if not specified the media directory is './' .\n"
            << " \t -p|--post-process bash|copy-local-to (optional) \n"
            << "\t\t Various post process task which can be launched ath the end of the simulation:\n"
            << "\t\t bash <int>|all <command> \n"
            << "\t\t\t <command>: specifies a quoted bash command as \"rm -r ./dir/\" \n"
            << "\t\t\t <int> is the rank number on which the bash command is launched, \n"
            << "\t\t\t 'all' specifies all process who execute this command \n"
            << "\t\t copy-local-to <path> \n"
            << "\t\t\t Copies the local directory which is process specific to another folder at <path> \n"
            << " \t -h|--help \n"
            << "\t\t Prints this help" << std::endl;
    }
};

// Global implementation for ApplicationCLOptions::PostProcessTask
std::ostream& operator<<(std::ostream& s, const ApplicationCLOptions::PostProcessTask& p);

#endif

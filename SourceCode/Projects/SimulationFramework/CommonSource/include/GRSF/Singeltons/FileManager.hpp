#ifndef GRSF_Singeltons_FileManager_hpp
#define GRSF_Singeltons_FileManager_hpp

#include <vector>
#include <string>
#include <map>
#include <fstream>

//#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include "GRSF/Common/Singleton.hpp"

#include "GRSF/Common/CommonFunctions.hpp"

/**
* @ingroup Singeltons
* @brief The file manager which handles the generation of files and logs.
*/
class FileManager : public Utilities::Singleton<FileManager> {
public:

    /**
      globalDirPath = first simulation directory, localDirPath = process specific simulation directory
    */
    FileManager();
    FileManager(boost::filesystem::path globalDirPath, boost::filesystem::path localDirPath);
    ~FileManager();

    boost::filesystem::path getGlobalDirectoryPath();
    boost::filesystem::path getLocalDirectoryPath();

    boost::filesystem::path           getPathCurrentSimFolder();
    void                              setPathCurrentSimFolder(std::string file_name);

    std::set< boost::filesystem::path > getSimFolderList();
    std::set< boost::filesystem::path > getPathsSimFilesOfCurrentSimFolder();
    boost::filesystem::path             getPathSceneFileOfCurrentSimFolder();

    boost::filesystem::path getNewSimFolderPath(boost::filesystem::path directory, std::string folder_prefix);

    void updateFileList(boost::filesystem::path directory, bool with_SubDirs);



    boost::filesystem::path copyFile(boost::filesystem::path from, boost::filesystem::path to, bool overwrite = false);

private:
    void init(boost::filesystem::path globalDirPath, boost::filesystem::path localDirPath);

    void scanAllSimFolders(const boost::filesystem::path &directory, const std::string &prefix, const bool &with_SubDirs);
    void updateAllSimDataFiles(const boost::filesystem::path &directory, const bool &with_SubDirs);

    std::map< boost::filesystem::path, std::set<boost::filesystem::path> > m_SimFilePaths; ///< [parent folderPath , set of simfiles paths inside folderPath]
    std::set< boost::filesystem::path > m_SimFolderPaths; ///< folderPaths , keys into m_SimFilePaths;

    unsigned int m_folderIdCounter;
    boost::mutex m_busy_mutex;

    boost::filesystem::path m_selectedFolderPath; ///< current folderPath , key into m_SimFilePaths;

    boost::filesystem::path m_globalDirPath, m_localDirPath;

};

#endif

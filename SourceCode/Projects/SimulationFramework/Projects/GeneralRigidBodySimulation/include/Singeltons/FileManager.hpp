#ifndef FILE_MANAGER_HPP
#define FILE_MANAGER_HPP

#include <stdlib.h> 
#include <string>
#include <map>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/thread.hpp>
#include <OGRE/Ogre.h>

#include "TypeDefs.hpp"
#include "CommonFunctions.hpp"

/**
* @ingroup Singeltons
* @brief The file manager which handles the generation of files and logs.
*/
class FileManager : public Ogre::Singleton<FileManager>{
public:

  FileManager();
  ~FileManager();
  
  boost::filesystem::path getPathSimFileSelected();
  boost::filesystem::path getPathSceneFileSelected();
  void setPathSelectedSimFile(std::string file_name);
  
  boost::filesystem::path getSimFilePath(std::string file_name);

  Ogre::StringVector getSimFileNameList();
  
  boost::filesystem::path getNewSimFolderPath(boost::filesystem::path directory, std::string folder_prefix);
  void updateFileList(boost::filesystem::path directory, bool with_SubDirs);

  void copyFile(boost::filesystem::path from, boost::filesystem::path to, bool overwrite = false);

private:
  void scanAllSimFolders(const boost::filesystem::path &directory, const std::string &prefix, const bool &with_SubDirs);
  void updateAllSimDataFiles(const boost::filesystem::path &directory, const bool &with_SubDirs);
  
  std::map< boost::filesystem::path, boost::filesystem::path > m_SimFilePaths;
  Ogre::StringVector m_SimFileNames;

  unsigned int m_fileIdCounter;
  boost::mutex m_busy_mutex;

  boost::filesystem::path m_selectedFilePath;

  Ogre::Log * m_pAppLog;
};

#endif
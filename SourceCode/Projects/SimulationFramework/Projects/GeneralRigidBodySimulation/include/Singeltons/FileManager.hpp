﻿#ifndef FILE_MANAGER_HPP
#define FILE_MANAGER_HPP

#include <vector>
#include <string>
#include <map>
#include <fstream>

#include <boost/filesystem.hpp>

#include <boost/thread.hpp>
#include "Singleton.hpp"

#include "TypeDefs.hpp"
#include "CommonFunctions.hpp"



/**
* @ingroup Singeltons
* @brief The file manager which handles the generation of files and logs.
*/
class FileManager : public Utilities::Singleton<FileManager>{
public:

  FileManager(boost::filesystem::path globalPath="");
  ~FileManager();

  boost::filesystem::path getGlobalDirectoryPath();

  boost::filesystem::path getPathSimFileSelected();
  boost::filesystem::path getPathSceneFileSelected();
  void setPathSelectedSimFile(std::string file_name);

  boost::filesystem::path getSimFilePath(std::string file_name);

  std::vector<std::string> getSimFileNameList();

  boost::filesystem::path getNewSimFolderPath(boost::filesystem::path directory, std::string folder_prefix);
  void updateFileList(boost::filesystem::path directory, bool with_SubDirs);

  boost::filesystem::path copyFile(boost::filesystem::path from, boost::filesystem::path to, bool overwrite = false);

private:
  void scanAllSimFolders(const boost::filesystem::path &directory, const std::string &prefix, const bool &with_SubDirs);
  void updateAllSimDataFiles(const boost::filesystem::path &directory, const bool &with_SubDirs);

  std::map< boost::filesystem::path, boost::filesystem::path > m_SimFilePaths;
  std::vector<std::string> m_SimFileNames;

  unsigned int m_fileIdCounter;
  boost::mutex m_busy_mutex;

  boost::filesystem::path m_selectedFilePath;

  boost::filesystem::path m_globalDirPath;

};

#endif

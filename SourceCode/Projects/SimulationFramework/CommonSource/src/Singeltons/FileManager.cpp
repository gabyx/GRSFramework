#include "FileManager.hpp"


#include "LogDefines.hpp"

//=========================================================


//=========================================================

using namespace std;

FileManager::FileManager(boost::filesystem::path globalDirPath, boost::filesystem::path localDirPath){
   init(globalDirPath,localDirPath);
}

FileManager::FileManager(){
    init("","");
}

void FileManager::init(boost::filesystem::path globalDirPath, boost::filesystem::path localDirPath){
      m_fileIdCounter = 0;
      m_globalDirPath = globalDirPath;
      m_localDirPath = localDirPath;

      if(!m_globalDirPath.empty()){
          if(!exists(m_globalDirPath)){
              if(!create_directories(m_globalDirPath)){
                ERRORMSG("Global Directory Path in FileManager could not be created!");
              }
          }
      }

      if(!m_localDirPath.empty()){
          if(!exists(m_localDirPath)){
              if(!create_directories(m_localDirPath)){
                ERRORMSG("Local Directory Path in FileManager could not be created!");
              }
          }
      }

}

FileManager::~FileManager()
{
  DECONSTRUCTOR_MESSAGE
}

boost::filesystem::path FileManager::getGlobalDirectoryPath(){
    return m_globalDirPath;
}

boost::filesystem::path FileManager::getLocalDirectoryPath(){
    return m_localDirPath;
}

boost::filesystem::path FileManager::copyFile(boost::filesystem::path from, boost::filesystem::path to, bool overwrite){
   boost::mutex::scoped_lock l(m_busy_mutex);

   if(boost::filesystem::is_directory(to)){
      to /= from.filename();
   }
   boost::system::error_code err;
   if(overwrite){
      boost::filesystem::copy_file(from,to,boost::filesystem::copy_option::fail_if_exists,err);
   }else{
      boost::filesystem::copy_file(from,to,boost::filesystem::copy_option::overwrite_if_exists,err);
   }
   //m_pAppLog->logMessage(err.message());
   return to;
}

boost::filesystem::path FileManager::getNewSimFolderPath(boost::filesystem::path relDirectoryPath,  std::string folder_prefix)
{
  boost::mutex::scoped_lock l(m_busy_mutex);

  boost::filesystem::path directory = m_globalDirPath;
  directory /= relDirectoryPath;

  scanAllSimFolders(directory,folder_prefix,false);

  std::stringstream new_foldername;
  new_foldername << folder_prefix << m_fileIdCounter;
  directory /= new_foldername.str();

  if(!create_directories(directory)){
    return "";
  }
  return directory;
}

void FileManager::updateFileList(boost::filesystem::path relDirectoryPath, bool with_SubDirs = false)
{
  boost::mutex::scoped_lock l(m_busy_mutex);

  boost::filesystem::path directory = m_globalDirPath;
  directory /= relDirectoryPath;

  m_SimFilePaths.clear();
  m_SimFileNames.clear();
  updateAllSimDataFiles(directory,with_SubDirs);
}

std::vector<std::string> FileManager::getSimFileNameList()
{
  boost::mutex::scoped_lock l(m_busy_mutex);
  return m_SimFileNames;
}

boost::filesystem::path FileManager::getSimFilePath(std::string file_name)
{
  boost::mutex::scoped_lock l(m_busy_mutex);
  boost::filesystem::path name =  file_name;

  std::map<boost::filesystem::path,boost::filesystem::path>::iterator it;
    it= m_SimFilePaths.find(name);
    if(it != m_SimFilePaths.end()){
      return (*it).second;
    }
  return "";
}

void FileManager::updateAllSimDataFiles(const boost::filesystem::path &relDirectoryPath, const bool &with_SubDirs = false)
{

  boost::filesystem::path directory = m_globalDirPath;
  directory /= relDirectoryPath;

    m_fileIdCounter = 0;
  // Scan path for files and add them...
  using namespace boost::filesystem;

  if( exists( directory ) )
  {
    directory_iterator end ;
    for( directory_iterator iter(directory) ; iter != end ; ++iter ){
      if ( is_directory( *iter ) )
      {
        if( with_SubDirs ) updateAllSimDataFiles(*iter, with_SubDirs) ;
      }
      else
      {
        // Check for filename
        path prefix = iter->path().extension();
        if(prefix.string() == SIM_FILE_EXTENSION){

          // add file to the list
          m_SimFilePaths.insert(std::pair<path,path>(iter->path(),iter->path()));
          m_SimFileNames.push_back(iter->path().string());
        }
      }
    }
  }
}


void FileManager::scanAllSimFolders(const boost::filesystem::path &relDirectoryPath, const std::string & folder_prefix, const bool &with_SubDirs)
{

  boost::filesystem::path directory = m_globalDirPath;
  directory /= relDirectoryPath;

  m_fileIdCounter = 0;
  // Scan path for files and add them...
  using namespace boost::filesystem;

  if( exists( directory ) )
  {
    directory_iterator end ;
    for( directory_iterator iter(directory) ; iter != end ; ++iter ){

      if ( is_directory( *iter ) )
      {

         //// Get the number of the folder if name matches "<folder_prefix><number>"
         std::string name = iter->path().filename().string();
         std::string suffix = iter->path().extension().string();
         std::size_t found = name.find(folder_prefix);
         if(found!=string::npos){
           found += folder_prefix.length();
           int number_length = (((int)name.length()-(int)suffix.length() -1) -  (int)found) + 1;
           if( number_length >0){
             std::string number_string = name.substr(found, number_length);
             unsigned int numberId;
             if( Utilities::stringToType<unsigned int>(numberId,number_string, std::dec)){
               // Conversion worked
               if( m_fileIdCounter <= numberId){
                 m_fileIdCounter = numberId+1;
               }
             }
           }
         }

         //cout << iter->path().string() << " (dir)\n" ;
        //Recurse into subdirectories
        if( with_SubDirs ) scanAllSimFolders(*iter, folder_prefix, with_SubDirs);

      }
      else
      {
        //cout << iter->path().string() << " (file)\n" ;
        // Check for all .sim files and add to the list
        path suffix = iter->path().extension();
        if(suffix.string() == SIM_FILE_EXTENSION){
          // add file to the list
          std::map<boost::filesystem::path,boost::filesystem::path>::iterator it;
          it= m_SimFilePaths.find(iter->path().filename());
          if(it == m_SimFilePaths.end()){
            m_SimFilePaths.insert(std::pair<path,path>(iter->path().filename(),iter->path()));
            m_SimFileNames.push_back(iter->path().string());
          }

          std::string name = iter->path().filename().string();
          //m_pAppLog->logMessage("Found file " + name);
        }
      }
    }
  }

  // Execute CallBack to notify

}

boost::filesystem::path FileManager::getPathSimFileSelected()
{
  boost::mutex::scoped_lock l(m_busy_mutex);
  return m_selectedFilePath;
}
boost::filesystem::path FileManager::getPathSceneFileSelected()
{
  boost::mutex::scoped_lock l(m_busy_mutex);
  boost::filesystem::path simFilePath = m_selectedFilePath;
  if(simFilePath.empty()){
      return boost::filesystem::path();
  }

  boost::filesystem::path sceneFilePath = simFilePath.parent_path();
  std::string sceneFile = SIM_SCENE_FILE_NAME + std::string(".xml");
  sceneFilePath /= sceneFile;
  if(boost::filesystem::exists(sceneFilePath)){
      return sceneFilePath;
  }

  return boost::filesystem::path();
}


void FileManager::setPathSelectedSimFile( std::string file_name )
{
  boost::mutex::scoped_lock l(m_busy_mutex);
  boost::filesystem::path name =  file_name;

  std::map<boost::filesystem::path,boost::filesystem::path>::iterator it;
  it= m_SimFilePaths.find(name);
  if(it != m_SimFilePaths.end()){
    m_selectedFilePath = (*it).second;
  }
  else{
    m_selectedFilePath = boost::filesystem::path();
  }
}


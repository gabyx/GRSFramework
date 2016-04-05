// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include "GRSF/singeltons/FileManager.hpp"


#include "GRSF/common/LogDefines.hpp"

#include "GRSF/dynamics/general/MultiBodySimFile.hpp"

//=========================================================


//=========================================================

using namespace std;

FileManager::FileManager(boost::filesystem::path globalDirPath, boost::filesystem::path localDirPath) {
    init(globalDirPath,localDirPath);
}

FileManager::FileManager() {
    init("./","./");
}

void FileManager::init(boost::filesystem::path globalDirPath, boost::filesystem::path localDirPath) {
    using namespace boost::filesystem;

    m_folderIdCounter = 0;
    m_globalDirPath = globalDirPath;
    m_localDirPath = localDirPath;

    if(!m_globalDirPath.empty()) {
        if(!exists(m_globalDirPath)) {
            if(!create_directories(m_globalDirPath)) {
                ERRORMSG("Global Directory Path in FileManager could not be created!");
            }
        }
    }

    if(!m_localDirPath.empty()) {
        if(!exists(m_localDirPath)) {
            if(!create_directories(m_localDirPath)) {
                ERRORMSG("Local Directory Path in FileManager could not be created!");
            }
        }
    }

}

FileManager::~FileManager() {
    DESTRUCTOR_MESSAGE
}

boost::filesystem::path FileManager::getGlobalDirectoryPath() {
    return m_globalDirPath;
}

boost::filesystem::path FileManager::getLocalDirectoryPath() {
    return m_localDirPath;
}

boost::filesystem::path FileManager::copyFile(boost::filesystem::path from, boost::filesystem::path to, bool overwrite) {
    boost::mutex::scoped_lock l(m_busy_mutex);

    if(boost::filesystem::is_directory(to)) {
        to /= from.filename();
    }
    boost::system::error_code err;
    if(overwrite) {
        boost::filesystem::copy_file(from,to,boost::filesystem::copy_option::fail_if_exists,err);
    } else {
        boost::filesystem::copy_file(from,to,boost::filesystem::copy_option::overwrite_if_exists,err);
    }
    //m_pAppLog->logMessage(err.message());
    return to;
}

boost::filesystem::path FileManager::getNewSimFolderPath(boost::filesystem::path relDirectoryPath,  std::string folder_prefix) {
    boost::mutex::scoped_lock l(m_busy_mutex);

    boost::filesystem::path directory = m_globalDirPath;
    directory /= relDirectoryPath;

    scanAllSimFolders(directory,folder_prefix,false);

    std::stringstream new_foldername;
    new_foldername << folder_prefix << m_folderIdCounter;
    directory /= new_foldername.str();
    //std::cout << "RELDIR:" << relDirectoryPath << " FILEIDCOUNTER: "<< m_folderIdCounter <<" PREFIX: " << folder_prefix<< std::endl;
    if(!create_directories(directory)) {
        return boost::filesystem::path();
    }
    return directory;
}

void FileManager::updateFileList(boost::filesystem::path relDirectoryPath, bool with_SubDirs = false) {
    boost::mutex::scoped_lock l(m_busy_mutex);

    boost::filesystem::path directory = m_globalDirPath;
    directory /= relDirectoryPath;

    m_SimFilePaths.clear();
    m_SimFolderPaths.clear();
    updateAllSimDataFiles(directory,with_SubDirs);
}

std::set<boost::filesystem::path > FileManager::getSimFolderList() {
    boost::mutex::scoped_lock l(m_busy_mutex);
    return m_SimFolderPaths;
}

void FileManager::updateAllSimDataFiles(const boost::filesystem::path &directory, const bool &with_SubDirs = false) {

    m_folderIdCounter = 0;
    // Scan path for files and add them...
    using namespace boost::filesystem;

    if( exists( directory ) ) {
        directory_iterator end ;
        for( directory_iterator iter(directory) ; iter != end ; ++iter ) {
            if ( is_directory( *iter ) ) {
                if( with_SubDirs ) updateAllSimDataFiles(*iter, with_SubDirs) ;
            } else {
                // Check for filename
                path prefix = iter->path().extension();
                if(prefix.string() == SIM_FILE_EXTENSION) {
                    // add file to the list of the parent path
                    m_SimFilePaths[iter->path().parent_path()].insert(iter->path());
                    m_SimFolderPaths.insert(iter->path().parent_path());
                }
            }
        }
    }
}


void FileManager::scanAllSimFolders(const boost::filesystem::path &directory,
                                    const std::string & folder_prefix,
                                    const bool &with_SubDirs) {
    //std::cout << "directory: " << directory << std::endl;

    m_folderIdCounter = 0;
    // Scan path for files and add them...
    using namespace boost::filesystem;

    if( exists( directory ) ) {
        directory_iterator end ;
        for( directory_iterator iter(directory) ; iter != end ; ++iter ) {

            if ( is_directory( *iter ) ) {

                //// Get the number of the folder if name matches "<folder_prefix><number>"
                std::string name = iter->path().filename().string();
                //std::cout << "NAME: " << name << std::endl;
                std::string suffix = iter->path().extension().string();
                std::size_t found = name.find(folder_prefix);
                if(found!=string::npos) {
                    found += folder_prefix.length();
                    int number_length = (((int)name.length()-(int)suffix.length() -1) -  (int)found) + 1;
                    //std::cout << "number_length: " << number_length << std::endl;
                    if( number_length >0) {
                        std::string number_string = name.substr(found, number_length);
                        unsigned int numberId;
                        if( Utilities::stringToType<unsigned int>(numberId,number_string)) {
                            // Conversion worked
                            if( m_folderIdCounter <= numberId) {
                                m_folderIdCounter = numberId+1;
                            }
                        }
                    }
                }

                //cout << iter->path().string() << " (dir)\n" ;
                //Recurse into subdirectories
                if( with_SubDirs ) scanAllSimFolders(*iter, folder_prefix, with_SubDirs);

            }
        }
    }

    // Execute CallBack to notify

}

boost::filesystem::path FileManager::getPathCurrentSimFolder() {
    boost::mutex::scoped_lock l(m_busy_mutex);
    return m_selectedFolderPath;
}

std::set<boost::filesystem::path> FileManager::getPathsSimFilesOfCurrentSimFolder() {
    boost::mutex::scoped_lock l(m_busy_mutex);
    auto it = m_SimFilePaths.find(m_selectedFolderPath);
    if( it != m_SimFilePaths.end()){
        return it->second;
    }
    return std::set<boost::filesystem::path>();
}

boost::filesystem::path FileManager::getPathSceneFileOfCurrentSimFolder() {

    boost::mutex::scoped_lock l(m_busy_mutex);
    if(m_selectedFolderPath.empty()) {
        return boost::filesystem::path();
    }

    boost::filesystem::path sceneFilePath = m_selectedFolderPath;
    std::string sceneFile = SIM_SCENE_FILE_NAME + std::string(".xml");
    sceneFilePath /= sceneFile;
    if(boost::filesystem::exists(sceneFilePath)) {
        return sceneFilePath;
    }

    return boost::filesystem::path();
}


void FileManager::setPathCurrentSimFolder( std::string file_name ) {
    boost::mutex::scoped_lock l(m_busy_mutex);
    boost::filesystem::path name =  file_name;

    auto it= m_SimFilePaths.find(name);
    if(it != m_SimFilePaths.end()) {
        m_selectedFolderPath = (it->first);
    } else {
        m_selectedFolderPath = boost::filesystem::path();
    }
}


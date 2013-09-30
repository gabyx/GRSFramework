#ifndef StateRecorderProcess_HPP
#define  StateRecorderProcess_HPP

#include <string>
#include <sstream>

#include <cstring>
#include <cerrno>

#include <boost/unordered_map.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "FileManager.hpp"

#include "CommonFunctions.hpp"

#include "MultiBodySimFilePart.hpp"
#include "SimpleLogger.hpp"

#include "GetFileDescriptorInfo.hpp"


/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one MultiBodySimFile.
* @{
*/
template <typename TDynamicsSystemType>
class StateRecorderProcess {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemType::DynamicsSystemConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderProcess( unsigned int id = 0, unsigned int cacheSize = 100 * 1024 * 1024);
    ~StateRecorderProcess();

    void writeStates(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list);

    void setDirectoryPath(boost::filesystem::path dir_path);

    bool openFile(bool truncate = true);
    bool closeFile();

protected:

    boost::filesystem::path m_directoryPath; ///< The path where the sim body part file is opened!

    void getSimBodyPartFileName(std::stringstream & s);

    Logging::Log * m_pSimulationLog;

    MultiBodySimFilePart m_simFilePart;

    unsigned int m_accessId;

    unsigned long long m_cacheLimit;
    unsigned long long m_cachedBytes;
    char * m_cache;

};

/** @} */



template<typename TDynamicsSystemType>
StateRecorderProcess<TDynamicsSystemType>::StateRecorderProcess( unsigned int id) {

    m_accessId = id;

    //Check if LogManager is available
    Logging::LogManager * manager = Logging::LogManager::getSingletonPtr();
    m_pSimulationLog = manager->getLog("SimulationLog");
    if(!m_pSimulationLog) {
        // Log does not exist make a new standart log!
        boost::filesystem::path filePath = FileManager::getSingletonPtr()->getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "StateRecorderLog.log";
        m_pSimulationLog = manager->createLog("StateRecorderLog",true,true,filePath);
    }


    //Make new cache array
    m_cache = new cache[m_cacheLimit];

}

template<typename TDynamicsSystemType>
StateRecorderProcess<TDynamicsSystemType>::~StateRecorderProcess() {
    DECONSTRUCTOR_MESSAGE
    m_simFilePart.close();
}

template<typename TDynamicsSystemType>
void StateRecorderProcess<TDynamicsSystemType>::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}


template<typename TDynamicsSystemType>
bool StateRecorderProcess<TDynamicsSystemType>::openFile(bool truncate){
    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimBodyPartFileName(s);
    file /= s.str();


    if(!m_simFilePart->openWrite(file,truncate)){
        LOG(m_pSimulationLog,"---> StateRecorderBody:: Could not open SimFile: " << file.string() << std::endl;);
        LOG(m_pSimulationLog, pBodyFile->getErrorString() );
        return false;
    }
    if(truncate){
       LOG(m_pSimulationLog,"---> StateRecorderBody:: Added SimFile (truncated):" << file.string() << std::endl; );
    }
    else{
        LOG(m_pSimulationLog,"---> StateRecorderBody:: Added SimFile: " << file.string() << std::endl; );
    }
    return true;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::getSimBodyPartFileName(std::stringstream & s){
    s.str("");
    s <<"SimDataProcess" <<"-"<<m_accessId<<SIM_FILE_PART_EXTENSION;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::writeStates(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list){

    typename TDynamicsSystemType::RigidBodySimContainerType::const_iterator_ordered it;

    //iterate over all bodies ordered by id
    for(it = body_list.begin(); it != body_list.end(); it++){
        // dump the date into the file directly, it will use a cache anyway!


    }

}


template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::closeFile(){
    m_simFilePart.close();
    return true;
}

#endif


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
* @brief This is the StateRecorder class which records each body's states to one process own MultiBodySimFilePart.
* @{
*/
template <typename TDynamicsSystemType>
class StateRecorderProcess {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemType::DynamicsSystemConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderProcess( unsigned int id = 0, unsigned int bufferSize = 100 * 1024 * 1024);
    ~StateRecorderProcess();

    void write(PREC time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList);

    void setDirectoryPath(boost::filesystem::path dir_path);

    bool createSimFile(bool truncate = true);
    bool closeSimFile();

protected:

    boost::filesystem::path m_directoryPath; ///< The path where the sim body part file is opened!

    void getSimFilePartName(std::stringstream & s);

    Logging::Log * m_pSimulationLog;

    MultiBodySimFilePart m_binarySimFile;

    unsigned int m_accessId;

    unsigned long long m_bufferSize;

};

/** @} */



template<typename TDynamicsSystemType>
StateRecorderProcess<TDynamicsSystemType>::StateRecorderProcess( unsigned int id,  unsigned int bufferSize):
        m_binarySimFile(LayoutConfigType::LayoutType::NDOFqObj, LayoutConfigType::LayoutType::NDOFuObj, m_bufferSize)
{

    m_accessId = id;
    m_bufferSize = bufferSize;

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

}

template<typename TDynamicsSystemType>
StateRecorderProcess<TDynamicsSystemType>::~StateRecorderProcess() {
    DECONSTRUCTOR_MESSAGE
    m_binarySimFile.close();
}

template<typename TDynamicsSystemType>
void StateRecorderProcess<TDynamicsSystemType>::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}

template<typename TDynamicsSystemType>
bool StateRecorderProcess<TDynamicsSystemType>::createSimFile(bool truncate){
    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimFilePartName(s);
    file /= s.str();


    if(!m_binarySimFile.openWrite(file,truncate)){
        LOG(m_pSimulationLog,"---> StateRecorderBody:: Could not open SimFile: " << file.string() << std::endl;);
        LOG(m_pSimulationLog, m_binarySimFile.getErrorString() );
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
void StateRecorderProcess<TDynamicsSystemType>::getSimFilePartName(std::stringstream & s){
    s.str("");
    s <<"SimDataProcess" <<"-"<<m_accessId<<SIM_FILE_PART_EXTENSION;
}

template<typename TDynamicsSystemType>
void StateRecorderProcess<TDynamicsSystemType>::write(PREC time, const typename TDynamicsSystemType::RigidBodySimContainerType & bodyList){
    m_binarySimFile.write(time, bodyList);
}


template<typename TDynamicsSystemType>
bool StateRecorderProcess<TDynamicsSystemType>::closeSimFile(){
    m_binarySimFile.close();
    return true;
}

#endif


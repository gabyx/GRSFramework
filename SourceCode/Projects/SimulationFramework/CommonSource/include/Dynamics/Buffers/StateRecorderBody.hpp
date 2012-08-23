#ifndef StateRecorderBody_HPP
#define  StateRecorderBody_HPP

#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <boost/unordered_map.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "FileManager.hpp"

#include "CommonFunctions.hpp"

#include "MultiBodySimFile.hpp"
#include "SimpleLogger.hpp"

#include "RigidBody.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one MultiBodySimFile.
* @{
*/
template <typename TDynamicsSystemType>
class StateRecorderBody {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemType::DynamicsSystemConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderBody();
    ~StateRecorderBody();

    void writeStates(const typename TDynamicsSystemType::RigidBodySimPtrListType & body_list);

    bool openFiles(boost::filesystem::path dir_path, const typename TDynamicsSystemType::RigidBodySimPtrListType & body_list);
    bool closeFiles(unsigned int bodyId);

    void closeAllSimFiles();

protected:

    void getSimBodyFileName(typename TDynamicsSystemType::RigidBodyType *body, std::stringstream & s);

    Logging::Log * m_pSimulationLog;
    typedef boost::unordered_map< typename RigidBodyType::RigidBodyIdType, MultiBodySimFile<LayoutConfigType>* > FileMap;
    FileMap m_BinarySimFiles;

    unsigned int m_nSimBodies;
};

/** @} */



template<typename TDynamicsSystemType>
StateRecorderBody<TDynamicsSystemType>::StateRecorderBody() {

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
StateRecorderBody<TDynamicsSystemType>::~StateRecorderBody() {
    DECONSTRUCTOR_MESSAGE
    closeAllSimFiles();
}


template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::openFiles(boost::filesystem::path dir_path, const typename TDynamicsSystemType::RigidBodySimPtrListType & body_list)
{

    boost::filesystem::path file;
    std::stringstream s;

    // For everybody add a Sim File!
    typename TDynamicsSystemType::RigidBodySimPtrListType::const_iterator it;
    for(it = body_list.begin();it != body_list.end();it++){

        file = dir_path;

        getSimBodyFileName(it->get(),s);

        file /= s.str();


        MultiBodySimFile<LayoutConfigType>* pBodyFile = new MultiBodySimFile<LayoutConfigType>();
        std::pair<typename FileMap::iterator, bool> res = m_BinarySimFiles.insert(typename FileMap::value_type((*it)->m_id,pBodyFile));
        if(!res.second){
            m_pSimulationLog->logMessage("StateRecorderBody:: Sim file : " + file.string() + "already exists!");
            delete pBodyFile;
        }else{ //if insert took place

            // Do truncate
            if(!pBodyFile->openSimFileWrite(file,1,true)){
                m_pSimulationLog->logMessage("StateRecorderBody:: Could not open Sim file: " + file.string());
                m_pSimulationLog->logMessage(pBodyFile->getErrorString());
                delete pBodyFile;
                m_BinarySimFiles.erase(res.first);
                return false;
            }
             m_pSimulationLog->logMessage("StateRecorderBody:: Added Sim file: " + file.string());
        }


    }
    m_pSimulationLog->logMessage("StateRecorderBody:: Added and opened all Sim files");

    return true;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::getSimBodyFileName(typename TDynamicsSystemType::RigidBodyType *body,
                                                                std::stringstream & s){
    s.str("");
    s <<"SimData_Body_" <<"-"<<RigidBodyId::getProcessNr(body)<<"-"<< RigidBodyId::getBodyNr(body)<<SIM_FILE_EXTENSION;
}

void writeStates(const typename TDynamicsSystemType::RigidBodySimPtrListType & body_list){

}


template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::closeFiles(unsigned int bodyId){

    typename FileMap::iterator it = m_BinarySimFiles.find(bodyId);


    if(it != m_BinarySimFiles.end()){
        LOG(m_pSimulationLog, "StateRecorderBody:: Removed Sim file";);
    }else{
        LOG(m_pSimulationLog,"StateRecorderBody:: No Sim file to remove for Body Id: "<<bodyId);
        return false;
    }

    return true;
}


template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::closeAllSimFiles() {

    typename FileMap::iterator it;

    for(it=m_BinarySimFiles.begin();it!=m_BinarySimFiles.end();it++){
        it->second->closeSimFile();
        delete it->second;
    }
    m_BinarySimFiles.clear();
}



#endif

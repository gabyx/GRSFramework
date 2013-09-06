#ifndef StateRecorderBody_HPP
#define  StateRecorderBody_HPP

#include <string>
#include <sstream>


#include <boost/unordered_map.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "FileManager.hpp"

#include "CommonFunctions.hpp"

#include "MultiBodySimFile.hpp"
#include "SimpleLogger.hpp"


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

    void writeStates(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list);

    void addBody(RigidBodyType * body);
    void removeBody(RigidBodyType * body);

    void setDirectoryPath(boost::filesystem::path dir_path);

    bool openFiles(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list, bool truncate = false);
    bool openFile(RigidBodyType * body, bool truncate);
    bool closeFile(RigidBodyType * body);

    void closeAllSimFiles();

protected:

    boost::filesystem::path m_directoryPath; ///< The path where the sim body files are opened!

    void getSimBodyFileName(typename TDynamicsSystemType::RigidBodyType *body, std::stringstream & s);

    Logging::Log * m_pSimulationLog;
    typedef boost::unordered_map< typename RigidBodyType::RigidBodyIdType, MultiBodySimFile* > FileMap;
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
void StateRecorderBody<TDynamicsSystemType>::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}

template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::openFile(RigidBodyType * body, bool truncate){
    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimBodyFileName(body,s);
    file /= s.str();

    MultiBodySimFile* pBodyFile = new MultiBodySimFile(LayoutConfigType::LayoutType::NDOFqObj, LayoutConfigType::LayoutType::NDOFuObj);
    std::pair<typename FileMap::iterator, bool> res = m_BinarySimFiles.insert(typename FileMap::value_type(body->m_id,pBodyFile));
    if(!res.second){
        m_pSimulationLog->logMessage("---> StateRecorderBody:: Sim file : " + file.string() + "already exists!");
        delete pBodyFile;
    }else{
        // Do truncate
            if(!pBodyFile->openSimFileWrite(file,1,truncate)){
                m_pSimulationLog->logMessage("---> StateRecorderBody:: Could not open Sim file: " + file.string());
                m_pSimulationLog->logMessage(pBodyFile->getErrorString());
                delete pBodyFile;
                m_BinarySimFiles.erase(res.first);
                return false;
            }
            if(truncate){
                m_pSimulationLog->logMessage("---> StateRecorderBody:: Added Sim file (truncated):" + file.string() );
            }
            else{
                m_pSimulationLog->logMessage("---> StateRecorderBody:: Added Sim file: " + file.string() );

            }
    }
    return true;
}

template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::openFiles(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list,
                                                       bool truncate)
{
    // For every body add a Sim File!
    typename TDynamicsSystemType::RigidBodySimContainerType::const_iterator it;
    for(it = body_list.begin();it != body_list.end();it++){
        if(!openFile(*it,truncate)){
            m_pSimulationLog->logMessage("---> StateRecorderBody:: Opened all Sim files failed!");
            return false;
        }
    }
    m_pSimulationLog->logMessage("---> StateRecorderBody:: Opened all Sim files");

    return true;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::getSimBodyFileName(typename TDynamicsSystemType::RigidBodyType *body,
                                                                std::stringstream & s){
    s.str("");
    s <<"SimDataBody" <<"-"<<RigidBodyId::getGroupNr(body)<<"-"<< RigidBodyId::getBodyNr(body)<<SIM_FILE_EXTENSION;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::writeStates(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list){

    typename TDynamicsSystemType::RigidBodySimContainerType::const_iterator it;

    static DynamicsState<LayoutConfigType> dynState(1); // State for one Body only
        // Iterate over all bodies
    for(it = body_list.begin(); it != body_list.end(); it++){
        //Find Sim file in list
        typename FileMap::iterator fileIt = m_BinarySimFiles.find((*it)->m_id);

        if(fileIt == m_BinarySimFiles.end()){
            LOG(m_pSimulationLog, "StateRecorderBody:: Did not found SimFile for Body Id:"
                << RigidBodyId::getBodyIdString(*it)<< ". There is no SimFile corresponding to this body!" <<std::endl);
        }else{
            dynState.m_t = (*it)->m_pSolverData->m_t;
            InitialConditionBodies::applyBodyToRigidBodyState( *it , dynState.m_SimBodyStates[0]);
            *(fileIt->second) << dynState;

        }
    }

}


template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::closeFile(RigidBodyType * body){

    typename FileMap::iterator it = m_BinarySimFiles.find(body->m_id);


    if(it != m_BinarySimFiles.end()){
        it->second->closeSimFile();
        delete it->second;
        m_BinarySimFiles.erase(it);
        LOG(m_pSimulationLog, "---> StateRecorderBody:: Closed and removed Sim file" << std::endl;);
    }else{
        LOG(m_pSimulationLog,"---> StateRecorderBody:: No Sim file to remove for Body Id: "<<body->m_id<< std::endl);
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

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::addBody(RigidBodyType * body) {
    LOG(m_pSimulationLog, "---> StateRecorderBody:: Add body with id: " << RigidBodyId::getBodyIdString(body) <<std::endl;);
    bool res = openFile(body, false); // no truncate!
    LOGASSERTMSG(res, m_pSimulationLog, "---> StateRecorderBody:: Add body with id: " << RigidBodyId::getBodyIdString(body) <<"failed!");
}
template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::removeBody(RigidBodyType * body) {
    LOG(m_pSimulationLog, "---> StateRecorderBody:: Remove body with id: " << RigidBodyId::getBodyIdString(body) <<std::endl;);
    bool res = closeFile(body);
    LOGASSERTMSG(res, m_pSimulationLog, "---> StateRecorderBody:: Remove body with id: " << RigidBodyId::getBodyIdString(body) <<"failed!");
}



#endif

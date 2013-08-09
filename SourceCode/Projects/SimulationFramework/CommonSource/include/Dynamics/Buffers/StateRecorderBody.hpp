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

    void writeStates(const typename TDynamicsSystemType::RigidBodySimContainer & body_list);

    void addBody(RigidBodyType * body);
    void removeBody(RigidBodyType * body);

    bool openFiles(boost::filesystem::path dir_path, const typename TDynamicsSystemType::RigidBodySimContainer & body_list, bool truncate = false);
    bool closeFile(typename RigidBodyType::RigidBodyIdType);

    void closeAllSimFiles();

protected:

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
bool StateRecorderBody<TDynamicsSystemType>::openFiles(boost::filesystem::path dir_path,
                                                       const typename TDynamicsSystemType::RigidBodySimContainer & body_list,
                                                       bool truncate)
{

    boost::filesystem::path file;
    std::stringstream s;

    // For every body add a Sim File!
    typename TDynamicsSystemType::RigidBodySimContainer::const_iterator it;
    for(it = body_list.begin();it != body_list.end();it++){

        file = dir_path;

        getSimBodyFileName(*it,s);

        file /= s.str();


        MultiBodySimFile* pBodyFile = new MultiBodySimFile(LayoutConfigType::LayoutType::NDOFqObj, LayoutConfigType::LayoutType::NDOFuObj);
        std::pair<typename FileMap::iterator, bool> res = m_BinarySimFiles.insert(typename FileMap::value_type((*it)->m_id,pBodyFile));
        if(!res.second){
            m_pSimulationLog->logMessage("---> StateRecorderBody:: Sim file : " + file.string() + "already exists!");
            delete pBodyFile;
        }else{ //if insert took place

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


    }
    m_pSimulationLog->logMessage("---> StateRecorderBody:: Opened all Sim files");

    return true;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::getSimBodyFileName(typename TDynamicsSystemType::RigidBodyType *body,
                                                                std::stringstream & s){
    s.str("");
    s <<"SimData_Body_" <<"-"<<RigidBodyId::getGroupNr(body)<<"-"<< RigidBodyId::getBodyNr(body)<<SIM_FILE_EXTENSION;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::writeStates(const typename TDynamicsSystemType::RigidBodySimContainer & body_list){

    typename TDynamicsSystemType::RigidBodySimContainer::const_iterator it;

    static DynamicsState<LayoutConfigType> dynState(1); // State for one Body only
        // Iterate over all bodies
    for(it = body_list.begin(); it != body_list.end(); it++){
        //Find Sim file in list
        typename FileMap::iterator fileIt = m_BinarySimFiles.find((*it)->m_id);

        if(fileIt == m_BinarySimFiles.end()){
            LOG(m_pSimulationLog, "StateRecorderBody:: Did not found SimFile for Body Id:" << (*it)->m_id<< ". There is no SimFile corresponding to this body!" ;);
        }else{

            InitialConditionBodies::applyBodyToRigidBodyState( *(*it) , dynState.m_SimBodyStates[0]);
            *(fileIt->second) << dynState;

        }
    }

}


template<typename TDynamicsSystemType>
bool StateRecorderBody<TDynamicsSystemType>::closeFile(typename RigidBodyType::RigidBodyIdType bodyId){

    typename FileMap::iterator it = m_BinarySimFiles.find(bodyId);


    if(it != m_BinarySimFiles.end()){
        it->second->closeSimFile();
        delete it->second;
        m_BinarySimFiles.erase(it);
        LOG(m_pSimulationLog, "---> StateRecorderBody:: Closed and removed Sim file";);
    }else{
        LOG(m_pSimulationLog,"---> StateRecorderBody:: No Sim file to remove for Body Id: "<<bodyId);
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
    LOG(m_pSimulationLog, "---> StateRecorderBody:: Trying to add body with id: " << body->m_id <<std::endl;);
}
template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::removeBody(RigidBodyType * body) {
    LOG(m_pSimulationLog, "---> StateRecorderBody:: Remove body with id: " << body->m_id <<std::endl;);
}



#endif

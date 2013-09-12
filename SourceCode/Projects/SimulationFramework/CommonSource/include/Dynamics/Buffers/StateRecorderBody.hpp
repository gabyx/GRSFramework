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

    StateRecorderBody(bool logWriteAccess = false, unsigned int id = 0);
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
    void getSimBodyLogFileName(typename TDynamicsSystemType::RigidBodyType *body, std::stringstream & s);

    Logging::Log * m_pSimulationLog;
    typedef boost::unordered_map< typename RigidBodyType::RigidBodyIdType, MultiBodySimFile* > FileMap;
    FileMap m_BinarySimFiles;

    //open files to log the writes!
    typedef boost::unordered_map< typename RigidBodyType::RigidBodyIdType, std::ofstream* > FileMapLog;
    FileMapLog m_LogSimFiles;
    bool m_logWriteAccess;
    unsigned int m_accessId;

    unsigned int m_nSimBodies;
};

/** @} */



template<typename TDynamicsSystemType>
StateRecorderBody<TDynamicsSystemType>::StateRecorderBody(bool logWriteAccess, unsigned int id) {

    m_logWriteAccess = logWriteAccess;
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


        std::pair<typename FileMap::iterator, bool> res = m_BinarySimFiles.insert(typename FileMap::value_type(body->m_id,NULL));
        if(!res.second){
            m_pSimulationLog->logMessage("---> StateRecorderBody:: SimFile : " + file.string() + "already exists!");
        }else{
            // Do truncate
            MultiBodySimFile* pBodyFile = new MultiBodySimFile(LayoutConfigType::LayoutType::NDOFqObj, LayoutConfigType::LayoutType::NDOFuObj);
            res.first->second =  pBodyFile; // Set the file

                if(!pBodyFile->openSimFileWrite(file,1,truncate)){
                    m_pSimulationLog->logMessage("---> StateRecorderBody:: Could not open SimFile: " + file.string());
                    m_pSimulationLog->logMessage(pBodyFile->getErrorString());
                    delete pBodyFile;
                    m_BinarySimFiles.erase(res.first);
                    return false;
                }
                if(truncate){
                    m_pSimulationLog->logMessage("---> StateRecorderBody:: Added SimFile (truncated):" + file.string() );
                }
                else{
                    m_pSimulationLog->logMessage("---> StateRecorderBody:: Added SimFile: " + file.string() );

                }
        }



    if( m_logWriteAccess ){

        file = m_directoryPath;
        getSimBodyLogFileName(body,s);
        file /= s.str();

        // open the log file and append to it!
        auto res = m_LogSimFiles.insert(typename FileMapLog::value_type(body->m_id,NULL));

        if(!res.second){
            m_pSimulationLog->logMessage("---> StateRecorderBody:: LogSimFile : " + file.string() + "already exists!");
        }else{
            // Do append!
            std::ofstream * pLogFile = new std::ofstream();
            pLogFile->close(); //safty
            pLogFile->open(file.string(),std::ios::app | std::ios::out);
            res.first->second =  pLogFile; // Set the file

                if(pLogFile->fail()){
                    m_pSimulationLog->logMessage("---> StateRecorderBody:: Could not open LogSimFile: " + file.string());
                    delete pLogFile;
                    m_LogSimFiles.erase(res.first);
                    return false;
                }
                if(truncate){
                    m_pSimulationLog->logMessage("---> StateRecorderBody:: Added LogSimFile (truncated):" + file.string() );
                }
                else{
                    m_pSimulationLog->logMessage("---> StateRecorderBody:: Added LogSimFile: " + file.string() );

                }
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
void StateRecorderBody<TDynamicsSystemType>::getSimBodyLogFileName(typename TDynamicsSystemType::RigidBodyType *body,
                                                                std::stringstream & s){
    s.str("");
    s <<"SimDataBodyAccess" <<"-"<<RigidBodyId::getGroupNr(body)<<"-"<< RigidBodyId::getBodyNr(body)<<SIM_FILE_ACCESS_LOG_EXTENSION;
}

template<typename TDynamicsSystemType>
void StateRecorderBody<TDynamicsSystemType>::writeStates(const typename TDynamicsSystemType::RigidBodySimContainerType & body_list){

    typename TDynamicsSystemType::RigidBodySimContainerType::const_iterator it;

    static DynamicsState<LayoutConfigType> dynState(1); // state for one Body only
        // iterate over all bodies
    for(it = body_list.begin(); it != body_list.end(); it++){
        // find Sim file in list
        typename FileMap::iterator fileIt = m_BinarySimFiles.find((*it)->m_id);

        if(fileIt == m_BinarySimFiles.end()){
            LOG(m_pSimulationLog, "StateRecorderBody:: Did not found SimFile for Body Id:"
                << RigidBodyId::getBodyIdString(*it)<< ". There is no SimFile corresponding to this body!" <<std::endl);
        }else{
            dynState.m_t = (*it)->m_pSolverData->m_t;
            InitialConditionBodies::applyBodyToRigidBodyState( *it , dynState.m_SimBodyStates[0]);
            *(fileIt->second) << dynState;
        }

        if(m_logWriteAccess){
            // find LogSim file
            auto logfileIt = m_LogSimFiles.find((*it)->m_id);
            if(logfileIt == m_LogSimFiles.end()){
                LOG(m_pSimulationLog, "StateRecorderBody:: Did not found LogSimFile for Body Id:" << RigidBodyId::getBodyIdString(*it)<< ". There is no SimFile corresponding to this body!" <<std::endl);
            }else{
                *(logfileIt->second) << (*it)->m_pSolverData->m_t << "\t" << m_accessId  << std::endl;
            }
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
        LOG(m_pSimulationLog, "---> StateRecorderBody:: Closed and removed SimFile" << std::endl;);
    }else{
        LOG(m_pSimulationLog,"---> StateRecorderBody:: No Sim file to remove for Body Id: "<<body->m_id<< std::endl);
        return false;
    }

    if(m_logWriteAccess){
        auto it = m_LogSimFiles.find(body->m_id);

        if(it != m_LogSimFiles.end()){
            it->second->close();
            delete it->second;
            m_LogSimFiles.erase(it);
        }else{
           LOG(m_pSimulationLog,"---> StateRecorderBody:: No LogSimFile to remove for Body Id: "<<body->m_id<< std::endl);
           return false;
        }
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

    if(m_logWriteAccess){
        for(auto it=m_LogSimFiles.begin();it!=m_LogSimFiles.end();it++){
            it->second->close();
            delete it->second;
        }
        m_LogSimFiles.clear();
    }
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

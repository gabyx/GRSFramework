#ifndef GMSF_Dynamics_Buffers_StateRecorderBody_hpp
#define GMSF_Dynamics_Buffers_StateRecorderBody_hpp

#include <string>
#include <sstream>

#include <cstring>
#include <cerrno>

#include <unordered_map>

#include "GMSF/Common/LogDefines.hpp"
#include "GMSF/Common/TypeDefs.hpp"

#include "GMSF/Singeltons/FileManager.hpp"

#include "GMSF/Common/CommonFunctions.hpp"

#include "GMSF/Dynamics/General/MultiBodySimFile.hpp"
#include "GMSF/Common/SimpleLogger.hpp"

#include "GMSF/Common/GetFileDescriptorInfo.hpp"


#define SIM_FILE_ACCESS_LOG_EXTENSION ".dat" ///< File extension for the access log file.


/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one MultiBodySimFile.
* @{
*/

class StateRecorderBody {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderBody(bool logWriteAccess = false, unsigned int id = 0);
    ~StateRecorderBody();

    template<typename TRigidBodyContainer>
    void write(PREC time, const TRigidBodyContainer & body_list);

    // Functions which are used for a delegate in other classes
    void addBody(RigidBodyType * body);
    void removeBody(RigidBodyType * body);

    void setDirectoryPath(boost::filesystem::path dir_path);

    bool createSimFiles(const typename DynamicsSystemType::RigidBodySimContainerType & body_list, bool truncate = false);
    bool createSimFile(RigidBodyType * body, bool truncate);
    bool closeSimFile(RigidBodyType * body);

    void closeAll();

protected:

    boost::filesystem::path m_directoryPath; ///< The path where the sim body files are opened!

    void getSimBodyFileName(typename DynamicsSystemType::RigidBodyType *body, std::stringstream & s);
    void getSimBodyLogFileName(typename DynamicsSystemType::RigidBodyType *body, std::stringstream & s);

    Logging::Log * m_pSimulationLog;
    typedef std::unordered_map< RigidBodyIdType, MultiBodySimFile* > FileMap;
    FileMap m_BinarySimFiles;

    //open files to log the writes!
    typedef std::unordered_map< RigidBodyIdType, std::ofstream* > FileMapLog;
    FileMapLog m_LogSimFiles;
    bool m_logWriteAccess;
    unsigned int m_accessId;

    unsigned int m_nSimBodies;

};

/** @} */



StateRecorderBody::StateRecorderBody(bool logWriteAccess, unsigned int id) {

    m_logWriteAccess = logWriteAccess;
    m_accessId = id;

    //Check if LogManager is available
    Logging::LogManager & manager = Logging::LogManager::getSingleton();
    m_pSimulationLog = manager.getLog("SimulationLog");
    if(!m_pSimulationLog) {
        // Log does not exist make a new standart log!
        boost::filesystem::path filePath = FileManager::getSingleton().getLocalDirectoryPath();
        filePath /= GLOBAL_LOG_FOLDER_DIRECTORY;
        filePath /= "StateRecorderLog.log";
        m_pSimulationLog = manager.createLog("StateRecorderLog",true,true,filePath);
    }

    // Get status information about file descriptors ...
    std::stringstream s;
    getLimitOpenFiles(s);
    LOG(m_pSimulationLog,"---> StateRecorderBody:: File Descriptor Info: " << std::endl << "\t\t" << s.str() << std::endl;)


}


StateRecorderBody::~StateRecorderBody() {
    DECONSTRUCTOR_MESSAGE
    closeAll();
}


void StateRecorderBody::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}



bool StateRecorderBody::createSimFile(RigidBodyType * body, bool truncate){
    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimBodyFileName(body,s);
    file /= s.str();


        std::pair<typename FileMap::iterator, bool> res = m_BinarySimFiles.insert(typename FileMap::value_type(body->m_id,nullptr));
        if(!res.second){
            LOG(m_pSimulationLog, "---> StateRecorderBody:: SimFile : " << file.string() << "already exists!");
        }else{
            // Do truncate
            MultiBodySimFile* pBodyFile = new MultiBodySimFile();
            res.first->second =  pBodyFile; // Set the file

                if(!pBodyFile->openWrite(file,LayoutConfigType::LayoutType::NDOFqBody, LayoutConfigType::LayoutType::NDOFuBody,1,truncate)){
                    LOG(m_pSimulationLog,"---> StateRecorderBody:: Could not open SimFile: " << file.string() <<
                        " number of files open: " << m_BinarySimFiles.size() << std::endl;
                        );
                    LOG(m_pSimulationLog, pBodyFile->getErrorString() );
                    delete pBodyFile;
                    m_BinarySimFiles.erase(res.first);
                    return false;
                }
                if(truncate){
                   LOG(m_pSimulationLog,"---> StateRecorderBody:: Added SimFile (truncated):" << file.string() << std::endl; );
                }
                else{
                    LOG(m_pSimulationLog,"---> StateRecorderBody:: Added SimFile: " << file.string() << std::endl; );

                }
        }



    if( m_logWriteAccess ){

        file = m_directoryPath;
        getSimBodyLogFileName(body,s);
        file /= s.str();

        // open the log file and append to it!
        auto res = m_LogSimFiles.insert(typename FileMapLog::value_type(body->m_id,nullptr));

        if(!res.second){
            LOG(m_pSimulationLog,"---> StateRecorderBody:: LogSimFile : " << file.string() << "already exists!");
        }else{
            // Do append!
            std::ofstream * pLogFile = new std::ofstream();
            pLogFile->close(); //safty
            pLogFile->open(file.string(),std::ios::app | std::ios::out);
            res.first->second =  pLogFile; // Set the file

                if(pLogFile->fail()){
                    LOG(m_pSimulationLog, "---> StateRecorderBody:: Could not open LogSimFile: " << file.string() << ", error: " << std::strerror(errno) );
                    delete pLogFile;
                    m_LogSimFiles.erase(res.first);
                    return false;
                }
                if(truncate){
                    LOG(m_pSimulationLog,"---> StateRecorderBody:: Added LogSimFile (truncated):" << file.string() );
                }
                else{
                    LOG(m_pSimulationLog,"---> StateRecorderBody:: Added LogSimFile: " << file.string() );

                }
        }

    }


    return true;
}


bool StateRecorderBody::createSimFiles(const typename DynamicsSystemType::RigidBodySimContainerType & body_list,
                                                       bool truncate)
{
    // For every body add a Sim File!
    typename DynamicsSystemType::RigidBodySimContainerType::const_iterator it;
    for(it = body_list.begin();it != body_list.end();it++){
        if(!createSimFile(*it,truncate)){
            m_pSimulationLog->logMessage("---> StateRecorderBody:: Opened all Sim files failed!");
            return false;
        }
    }
    m_pSimulationLog->logMessage("---> StateRecorderBody:: Opened all Sim files");

    return true;
}


void StateRecorderBody::getSimBodyFileName(typename DynamicsSystemType::RigidBodyType *body,
                                                                std::stringstream & s){
    s.str("");
    s <<"SimDataBody" <<"-"<<RigidBodyId::getGroupNr(body)<<"-"<< RigidBodyId::getBodyNr(body)<<SIM_FILE_EXTENSION;
}


void StateRecorderBody::getSimBodyLogFileName(typename DynamicsSystemType::RigidBodyType *body,
                                                                std::stringstream & s){
    s.str("");
    s <<"SimDataBody" <<"-"<<RigidBodyId::getGroupNr(body)<<"-"<< RigidBodyId::getBodyNr(body)<<"-Access"<< SIM_FILE_ACCESS_LOG_EXTENSION;
}


template<typename TRigidBodyContainer>
void StateRecorderBody::write(PREC time, const TRigidBodyContainer & body_list){

        // iterate over all bodies
    for(auto it = body_list.begin(); it != body_list.end(); it++){
        // find Sim file in list
        typename FileMap::iterator fileIt = m_BinarySimFiles.find((*it)->m_id);

        LOGASSERTMSG(fileIt != m_BinarySimFiles.end(), m_pSimulationLog, "StateRecorderBody:: Did not found SimFile for Body Id:"
            << RigidBodyId::getBodyIdString(*it)<< ". There is no SimFile corresponding to this body!" <<std::endl);


        fileIt->second->write(time, it, std::next(it) );


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



bool StateRecorderBody::closeSimFile(RigidBodyType * body){

    typename FileMap::iterator it = m_BinarySimFiles.find(body->m_id);


    if(it != m_BinarySimFiles.end()){
        it->second->close();
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



void StateRecorderBody::closeAll() {

    typename FileMap::iterator it;

    for(it=m_BinarySimFiles.begin();it!=m_BinarySimFiles.end();it++){
        it->second->close();
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


void StateRecorderBody::addBody(RigidBodyType * body) {
    LOG(m_pSimulationLog, "---> StateRecorderBody:: Add body with id: " << RigidBodyId::getBodyIdString(body) <<std::endl;);
    bool res = createSimFile(body, false); // no truncate!
    LOGASSERTMSG(res, m_pSimulationLog, "---> StateRecorderBody:: Add body with id: " << RigidBodyId::getBodyIdString(body) <<"failed!"
            << ", StateRecorderBody has " << m_BinarySimFiles.size() + m_LogSimFiles.size() << " files open currently!"
            );
}

void StateRecorderBody::removeBody(RigidBodyType * body) {
    LOG(m_pSimulationLog, "---> StateRecorderBody:: Remove body with id: " << RigidBodyId::getBodyIdString(body) <<std::endl;);
    bool res = closeSimFile(body);
    LOGASSERTMSG(res, m_pSimulationLog, "---> StateRecorderBody:: Remove body with id: " << RigidBodyId::getBodyIdString(body) <<"failed!"
            << ", StateRecorderBody has " << m_BinarySimFiles.size() + m_LogSimFiles.size()<< " files open currently!"
            );
}



#endif

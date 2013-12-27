#ifndef StateRecorderMPI_HPP
#define  StateRecorderMPI_HPP

#include <mpi.h>

#include <string>
#include <sstream>

#include <cstring>
#include <cerrno>

#include "AssertionDebug.hpp"
#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "FileManager.hpp"

#include "CommonFunctions.hpp"

#include "SimpleLogger.hpp"

#include "MPIInformation.hpp"

#include "MultiBodySimFileMPI.hpp"

/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one process own MultiBodySimFilePart.
* @{
*/

class StateRecorderMPI {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderMPI(unsigned int nSimBodies);
    ~StateRecorderMPI();

    //Each process writes its stuff at a specific offset
    void write(PREC time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList);

    void setDirectoryPath(boost::filesystem::path dir_path);

    //Collective operation
    bool createSimFile(bool truncate = true);
    //Collective operation
    bool closeAll();

protected:

    typedef MPILayer::ProcessInformation ProcessInfoType;
    ProcessInfoType m_processInfo;

    boost::filesystem::path m_directoryPath; ///< The path where the sim body part file is opened!

    void getSimBodyFileName(std::stringstream & s);

    Logging::Log * m_pSimulationLog;

    unsigned int m_nSimBodies;
    MultiBodySimFileMPI m_fh;

    //Write buffer
    //std::vector<char> m_writebuffer;
    //    boost::iostreams::back_insert_device<std::vector<char> >m_ins; // is initialized first
    //    boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > m_stream;  //is initialized second
    //    boost::archive::binary_oarchive m_oa; // is initialized third

};

/** @} */




StateRecorderMPI::StateRecorderMPI(unsigned int nSimBodies):
    m_fh(LayoutConfigType::LayoutType::NDOFqBody, LayoutConfigType::LayoutType::NDOFuBody), m_nSimBodies(nSimBodies)
//    ,m_ins(m_writebuffer),
//    m_stream(m_ins),
//    m_oa( m_stream,boost::archive::no_codecvt | boost::archive::no_header)
{
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

     //write buffer
    //m_writebuffer.reserve(4000*((LayoutConfigType::LayoutType::NDOFqObj + LayoutConfigType::LayoutType::NDOFuObj)*sizeof(double)+1*sizeof(typename RigidBodyType::RigidBodyIdType))); // reserved for 5000 bodies :)

    // id: 0 -10  Process 0
    // id: 11-20 Process 1
    //...
    // id: 1001-1010 Process N-1

//    // each processor (except the last) contains exactly m_nBodiesPerProc bodies which he writes
//    m_nBodiesPerProc =  m_nSimBodies / m_processInfo.getNProcesses();
//
//    m_nRecvBodies = m_nBodiesPerProc;
//
//    //last rank nimmt den rest der bodies
//    if( m_processInfo.getRank() == m_processInfo.getNProcesses()-1  ){
//        m_nRecvBodies += m_nSimBodies % m_processInfo.getNProcesses()
//    }



    //receive buffer
    // this buffer should stays the same and contains only the bodies in the range!
    //m_recvbuffer.reserve(m_nRecvBodies * ((7+6)*sizeof(double)+1*sizeof(typename RigidBodyType::RigidBodyIdType)) )

//    // for each process
//    m_process_send_body_count.assign(m_processInfo.getNProcesses(),0);
//    m_process_send_body_displ.assign(m_processInfo.getNProcesses(),0);
//
//    m_process_recv_body_count.assign(m_processInfo.getNProcesses(),0);
//    m_process_recv_body_displ.assign(m_processInfo.getNProcesses(),0);


}


StateRecorderMPI::~StateRecorderMPI() {



}


void StateRecorderMPI::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}


bool StateRecorderMPI::createSimFile(bool truncate){

    boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimBodyFileName(s);
    file /= s.str();

    LOG(m_pSimulationLog,"MPI> StateRecorderMPI: Try Opened File : " << file << std::endl)
    //Collective (all processes do this)
    bool res = m_fh.openWrite(m_processInfo.getMPIComm(),file,m_nSimBodies);
    if(!res){
        ERRORMSG(m_fh.getErrorString());
    }
    LOG(m_pSimulationLog,"MPI> StateRecorderMPI: Opened File : " << file << std::endl)
    return true;
}


void StateRecorderMPI::getSimBodyFileName(std::stringstream & s){
    s.str("");
    s <<"SimDataMPIUnordered"<<SIM_FILE_MPI_EXTENSION;
}


void StateRecorderMPI::write(PREC time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList){

    m_fh.write(time,bodyList);

}



bool StateRecorderMPI::closeAll(){
   m_fh.close();
}

#endif



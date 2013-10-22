#ifndef StateRecorderProcess_HPP
#define  StateRecorderProcess_HPP

#include <mpi.h>

#include <string>
#include <sstream>

#include <cstring>
#include <cerrno>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/unordered_map.hpp>

#include "LogDefines.hpp"
#include "TypeDefs.hpp"

#include "FileManager.hpp"

#include "CommonFunctions.hpp"

#include "SimpleLogger.hpp"

#include "MPIInformation.hpp"



/**
* @ingroup StatesAndBuffers
* @brief This is the StateRecorder class which records each body's states to one process own MultiBodySimFilePart.
* @{
*/
template <typename TDynamicsSystemType>
class StateRecorderMPI {
public:
    typedef TDynamicsSystemType DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemType::DynamicsSystemConfig)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateRecorderMPI( unsigned int id = 0, unsigned int bufferSize = 100 * 1024 * 1024);
    ~StateRecorderMPI();

    void write(PREC time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList);

    void setDirectoryPath(boost::filesystem::path dir_path);

    bool openFile(bool truncate = true);
    bool closeFile();

protected:

    typedef MPILayer::ProcessInformation<DynamicsSystemType> ProcessInfoType;
    ProcessInfoType m_processInfo;

    boost::filesystem::path m_directoryPath; ///< The path where the sim body part file is opened!

    void getSimFilePartName(std::stringstream & s);

    Logging::Log * m_pSimulationLog;

    std::vector<int> m_process_send_body_count;
    std::vector<int> m_process_send_body_displ;

    std::vector<int> m_process_recv_body_count;
    std::vector<int> m_process_recv_body_displ;

    std::vector<char> m_sendbuffer;
    boost::archive::binary_oarchive m_oa;

    std::vector<char> m_recvbuffer;


    unsigned int m_nSimBodies, m_nBodiesPerProc, m_nRecvBodies;
    std::pair<unsigned int , unsigned int> m_bodyRange;

    MPI_File m_fh;
};

/** @} */



template<typename TDynamicsSystemType>
StateRecorderMPI<TDynamicsSystemType>::StateRecorderMPI(unsigned int nSimBodies):
    m_nSimBodies(nSimBodies)
{
    // id: 0 -10  Process 0
    // id: 11-20 Process 1
    //...
    // id: 1001-1010 Process N-1

    // each processor (except the last) contains exactly m_nBodiesPerProc bodies which he writes
    m_nBodiesPerProc =  m_nSimBodies / m_processInfo.getNProcesses();

    m_nRecvBodies = m_nBodiesPerProc;

    //last rank nimmt den rest der bodies
    if( m_processInfo.getRank() == m_processInfo.getNProcesses()-1  ){
        m_nRecvBodies += m_nSimBodies % m_processInfo.getNProcesses()
    }

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

    //send buffer

    m_sendbuffer.reserve(4000*((7+6)*sizeof(double)+1*sizeof(typename RigidBodyType::RigidBodyIdType))); // reserved for 5000 bodies :)

    boost::iostreams::back_insert_device<std::vector<char> > inserter(m_sendbuffer);
    boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > s(inserter);
    m_oa =  boost::archive::binary_oarchive(s, boost::archive::no_codecvt | boost::archive::no_header);

    //receive buffer
    // this buffer should stays the same and contains only the bodies in the range!
    m_recvbuffer.reserve(m_nRecvBodies * ((7+6)*sizeof(double)+1*sizeof(typename RigidBodyType::RigidBodyIdType)) )

    // for each process
    m_process_send_body_count.assign(m_processInfo.getNProcesses(),0);
    m_process_send_body_displ.assign(m_processInfo.getNProcesses(),0);

    m_process_recv_body_count.assign(m_processInfo.getNProcesses(),0);
    m_process_recv_body_displ.assign(m_processInfo.getNProcesses(),0);
}

template<typename TDynamicsSystemType>
StateRecorderMPI<TDynamicsSystemType>::~StateRecorderMPI() {

    MPI_File_close(&m_fh);

}

template<typename TDynamicsSystemType>
void StateRecorderMPI<TDynamicsSystemType>::setDirectoryPath(boost::filesystem::path dir_path){
    m_directoryPath = dir_path;
}

template<typename TDynamicsSystemType>
bool StateRecorderMPI<TDynamicsSystemType>::openFile(bool truncate){

     boost::filesystem::path file;
    std::stringstream s;

    file = m_directoryPath;
    getSimBodyFileName(body,s);
    file /= s.str();

    MPI_File_open(MPI_COMM_WORLD, file.str().c_str(), MPI_MODE_CREATE | MPI_MODE_RDWR, MPI_INFO_NULL, &m_fh);

    // write header

    for(int i=0; i<SIM_FILE_SIGNATURE_LENGTH; i++) {
        *this << m_simFileSignature[i];
    }

    *this << (unsigned int)m_nSimBodies << (unsigned int)m_nDOFqObj << (unsigned int)m_nDOFuObj; // Precision output is always double!


}

template<typename TDynamicsSystemType>
void StateRecorderMPI<TDynamicsSystemType>::getSimFilePartName(std::stringstream & s){
    s.str("");
    s <<"SimDataProcess" <<"-"<<m_accessId<<SIM_FILE_EXTENSION;
}

template<typename TDynamicsSystemType>
void StateRecorderMPI<TDynamicsSystemType>::write(PREC time, const typename TDynamicsSystemType::RigidBodySimContainerType & bodyList){

    // Copy all data into the send binary buffer
    unsigned int bodyIdxG = 0;
    unsigned int bodyCountL = 0;
    unsigned int processIdx = 0;
    unsigned int processIdxLast = 0;

    //clear for each process
    m_process_send_body_count.assign(m_processInfo.getNProcesses(),0);
    m_process_send_body_displ.assign(m_processInfo.getNProcesses(),0);


    for(auto it = bodyList.beginOrdered(); it!= bodyList.endOrdered();it++){
        bodyCountL++

        m_oa << (*it)->m_id;
        serializeEigen(m_oa, (*it)->get_q());
        serializeEigen(m_oa, (*it)->get_u());

        // calculate  belonging process
        processIdx = std::min( (unsigned int)( RigidBodyId::getBodyNr(*it) / m_nBodiesPerProc), m_processInfo.getNProcesses()-1 );
        // at first body update the last process idx
        if(bodyIdxG == 0){
            processIdxLast = processIdx;
        }

        if(processIdxLast != processIdx){
            ASSERTMSG(processIdxLast > processIdx,"ProcessIdx needs to be bigger?, is the body list ordered by id?");

            // We are at a boundary
            ASSERTMSG(processIdx < m_processInfo.getNProcesses(),"ProcessIdx not in range");
            //Write the new index
            m_process_send_body_displ[processIdx] = bodyIdxG;
            //Write the count
            m_process_send_body_count[processIdxLast] = bodyCountL-1;

        }

        bodyIdxG++;
    }

    // Distribute the counts
    MPI_Alltoall(&m_process_send_body_count[0], 1, MPI_INT, &m_process_recv_body_count[0], 1,MPI_INT,MPI_COMM_WORLD);

    bool sorted = false;


    if(sorted){

        //Count the counts (should match up to m_nBodiesPerProc)
        int size = std::accumulate(m_process_recv_body_count.begin(),m_process_recv_body_count.end(),0);
        ASSERTMSG(size == m_nRecvBodies, "Process with rank: "<<m_processInfo.getRank() << " receives " <<
                  size << "body states instead of " << m_nRecvBodies);

        // Calculate the displacements in the recv buffer from the m_process_recv_body_count
        m_process_recv_body_disp[0] = 0;
        for(int i = 0; i < m_process_recv_body_count.size()-1; i++){
            m_process_recv_body_disp[i+1] = m_process_recv_body_disp[i] + m_process_recv_body_count[i];
        }

        // Allocate enough space for all m_nRecvBodies states
        m_recvbuffer.clear() // Clear, we deserialize into these


        // TODO
    }else{
        // Write out all states unsorted at the right offset



    }

}


template<typename TDynamicsSystemType>
bool StateRecorderMPI<TDynamicsSystemType>::closeFile(){

}

#endif



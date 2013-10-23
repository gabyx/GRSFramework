#ifndef MultiBodySimFileMPI_hpp
#define MultiBodySimFileMPI_hpp


#include <type_traits>
#include <fstream>

#include <boost/filesystem.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <StaticAssert.hpp>

#include "RigidBodyContainer.hpp"
#include "MultiBodySimFileIOHelpers.hpp"

#define SIM_FILE_MPI_SIGNATURE_LENGTH 4
#define SIM_FILE_MPI_SIGNATURE {'M','B','S','F'}
#define SIM_FILE_MPI_EXTENSION ".simmpi"

template <typename TDynamicsSystemType>
class MultiBodySimFileMPI {
public:

    typedef TDynamicsSystemType DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemType::DynamicsSystemConfig)

    MultiBodySimFileMPI(unsigned int nDOFqObj, unsigned int nDOFuObj);
    ~MultiBodySimFileMPI();

    bool openWrite(MPI_Comm comm,  const boost::filesystem::path & file_path,   const unsigned int nSimBodies,  bool truncate = true);
    void close();

    inline void write(double time, const std::vector<char> & bytes, unsigned int nBodies);

    std::string getErrorString() {
        return m_errorString.str();
    }
private:

    // Communicator which is used to write the file in parallel
    MPI_Comm m_comm;
    int m_rank; // rank in the communicator;
    int m_processes;

    MPI_Datatype m_dynamicStateTypeMPI;
    MPI_File m_file_handle;                      ///< The file stream which represents the binary data.

    static const char m_simFileSignature[SIM_FILE_MPI_SIGNATURE_LENGTH]; ///< The .sim file header.

    void writeHeader();
    boost::filesystem::path m_filePath;

    void setByteLengths(const unsigned int nSimBodies);
    unsigned int m_nSimBodies;
    std::streamoff m_nBytesPerState; ///< m_nSimBodies*(id,q,u) + time
    std::streamoff m_nBytesPerQ, m_nBytesPerU;

    unsigned int m_nStates;

    unsigned int m_nDOFuObj, m_nDOFqObj;
    const  std::streamoff m_nBytesPerQObj ;
    const  std::streamoff m_nBytesPerUObj ;
    const  std::streamoff m_nBytesPerBody; ///< id,q,u
    static const  std::streamoff m_headerLength = (3*sizeof(unsigned int) + SIM_FILE_MPI_SIGNATURE_LENGTH*sizeof(char));

    MultiBodySimFileMPI & operator =(const MultiBodySimFileMPI & file);



    std::stringstream m_errorString;

    bool mpiSucceded(int err){
        m_errorString.str("");
        if(err != MPI_SUCCESS){
            char * string; int length;
            MPI_Error_string( err , string, &length );
            m_errorString << string;
            return false;
        }
        return true;
    }
};


template <typename TDynamicsSystemType>
const char MultiBodySimFileMPI<TDynamicsSystemType>::m_simFileSignature[SIM_FILE_MPI_SIGNATURE_LENGTH] = SIM_FILE_MPI_SIGNATURE;



template <typename TDynamicsSystemType>
MultiBodySimFileMPI<TDynamicsSystemType>::MultiBodySimFileMPI(unsigned int nDOFqObj, unsigned int nDOFuObj)
    : m_nBytesPerQObj(nDOFqObj*sizeof(double)),
      m_nBytesPerUObj(nDOFuObj*sizeof(double)),
      m_nBytesPerBody(m_nBytesPerQObj + m_nBytesPerUObj + 1*sizeof(typename RigidBodyType::RigidBodyIdType))
{
    m_nDOFuObj = nDOFuObj;
    m_nDOFqObj = nDOFqObj;

    m_nStates = 0;

    m_nBytesPerState =0;
    m_nBytesPerU = 0;
    m_nBytesPerQ = 0;
    m_nSimBodies = 0;

    m_filePath = boost::filesystem::path();
    //Make datatype
    int error = MPI_Type_contiguous(m_nBytesPerBody ,MPI_BYTE,&m_dynamicStateTypeMPI);
    ASSERTMPIERROR(error,"Type contignous");
    error = MPI_Type_commit(&m_dynamicStateTypeMPI);
    ASSERTMPIERROR(error, "Type commit");
    int size;
    error = MPI_Type_size(m_dynamicStateTypeMPI,&size);
    ASSERTMPIERROR(error, "Type size");
    ASSERTMSG(size == m_nBytesPerBody, "MPI type has not the same byte size");
}



template <typename TDynamicsSystemType>
MultiBodySimFileMPI<TDynamicsSystemType>::~MultiBodySimFileMPI(){
    int error = MPI_Type_free(&m_dynamicStateTypeMPI);
    ASSERTMPIERROR(error, "Type free");
}

template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::close(){
     int err = MPI_File_close(&m_file_handle);
     ASSERTMPIERROR(err, "File close");
}


template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::write(double time, const std::vector<char> & bytes, unsigned int nBodies){

    //bytes need to be a multiple of the bytes for one body state
    ASSERTMSG(bytes.size() % ( m_nBytesPerBody ) == 0, bytes.size() << " bytes not a multiple of " << m_nBytesPerBody << " bytes");

    std::vector<int> nbodiesPerProc(m_processes);
    int err = MPI_Allgather(&nBodies,1,MPI_INT,&nbodiesPerProc[0] , 1, MPI_INT, m_comm);
    ASSERTMPIERROR(err, "gather");
    //Calculate our offset
    unsigned int offset = 0;
    if(m_rank != 0 ){
        offset = std::accumulate(nbodiesPerProc.begin(),nbodiesPerProc.begin() + m_rank - 1 , 0);
    }else{
        //write time
        MPI_Status s;
        int err = MPI_File_write_shared(m_file_handle,&time,1,MPI_DOUBLE,&s);
        ASSERTMPIERROR(err, "Write time");
    }

    // Use shared file pointer to write ordered stuff
    MPI_Status s;
    err = MPI_File_write_ordered(m_file_handle,const_cast<void*>((const void*)&bytes[0]),nBodies,m_dynamicStateTypeMPI,&s);
    ASSERTMPIERROR(err, "orederd write");
}

template <typename TDynamicsSystemType>
void  MultiBodySimFileMPI<TDynamicsSystemType>::writeHeader() {

    if( m_rank == 0){
        char * header = new char[m_headerLength];
        char * p = header;

        memcpy((void*)p,&m_simFileSignature,sizeof(char)*SIM_FILE_MPI_SIGNATURE_LENGTH);
        p += sizeof(char)*SIM_FILE_MPI_SIGNATURE_LENGTH;

        unsigned int t = m_nSimBodies;
        memcpy((void*)p,&t,sizeof(unsigned int));
        p += sizeof(unsigned int);

        t = m_nDOFqObj;
        memcpy((void*)p,&t,sizeof(unsigned int));
        p += sizeof(unsigned int);

        t = m_nDOFuObj;
        memcpy((void*)p,&t,sizeof(unsigned int));

        MPI_Status status;
        int err;
        err = MPI_File_write(m_file_handle,header,m_headerLength,MPI_BYTE,&status);
        ASSERTMPIERROR(err,"");

        delete[] header;
    }
}

template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::setByteLengths(const unsigned int nSimBodies) {
    m_nBytesPerState = nSimBodies*(m_nBytesPerUObj + m_nBytesPerQObj) + 1*sizeof(double);
    m_nSimBodies = nSimBodies;
}

template <typename TDynamicsSystemType>
bool MultiBodySimFileMPI<TDynamicsSystemType>::openWrite(MPI_Comm comm, const boost::filesystem::path &file_path, const unsigned int nSimBodies, bool truncate) {

    m_comm = comm;
    MPI_Comm_rank(m_comm, &m_rank);
    MPI_Comm_size(m_comm, &m_processes);

    setByteLengths(nSimBodies);


    std::string filepath_tmp =  file_path.string();
    char * file_path_c = const_cast<char *>(filepath_tmp.c_str());

    if(truncate) {
        int err = MPI_File_open(m_comm, file_path_c,  MPI_MODE_CREATE | MPI_MODE_RDWR, MPI_INFO_NULL, &m_file_handle);
        if(!mpiSucceded(err)){
            return false;
        }

        writeHeader();

    }else{
        int err = MPI_File_open(m_comm, file_path_c, MPI_MODE_APPEND | MPI_MODE_CREATE | MPI_MODE_RDWR, MPI_INFO_NULL, &m_file_handle);
        if(!mpiSucceded(err)){
            return false;
        }
    }

    // Set file view, for all processes skip header,
    char type[] = "native";
    int err = MPI_File_set_view(m_file_handle, (MPI_Offset)m_headerLength, m_dynamicStateTypeMPI,m_dynamicStateTypeMPI,type,MPI_INFO_NULL);
    if(!mpiSucceded(err)){
        return false;
    }

    return true;
}


#endif

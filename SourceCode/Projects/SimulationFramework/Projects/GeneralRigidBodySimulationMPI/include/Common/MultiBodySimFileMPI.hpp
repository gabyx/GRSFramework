#ifndef MultiBodySimFileMPI_hpp
#define MultiBodySimFileMPI_hpp


#include <type_traits>
#include <fstream>

#include <boost/filesystem.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "StaticAssert.hpp"
#include "CommonFunctions.hpp"
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

    inline void write(double time, const RigidBodyContainer<RigidBodyType> & bodyList);


    std::string getErrorString() {
        return m_errorString.str();
    }
private:
    typedef typename RigidBodyType::RigidBodyIdType RigidBodyIdType;

    // Communicator which is used to write the file in parallel, this communicator is duplicated from the one inputed
    // This prevents that accidentaly some other synchronization and stuff is
    MPI_Comm m_comm;
    int m_rank; // rank in the communicator;
    int m_processes;

    MPI_Datatype m_bodyStateTypeMPI;
    MPI_File m_file_handle;                      ///< The file stream which represents the binary data.

    static const char m_simFileSignature[SIM_FILE_MPI_SIGNATURE_LENGTH]; ///< The .sim file header.


    void writeBySharedPtr(double time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList);
    void writeByOffsets(double time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList);
    void writeByOffsets2(double time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList);

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

    std::vector<char> m_writebuffer;

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
    int error = MPI_Type_contiguous(m_nBytesPerBody ,MPI_BYTE,&m_bodyStateTypeMPI);
    ASSERTMPIERROR(error,"Type contignous");
    error = MPI_Type_commit(&m_bodyStateTypeMPI);
    ASSERTMPIERROR(error, "Type commit");
    int size;
    error = MPI_Type_size(m_bodyStateTypeMPI,&size);
    ASSERTMPIERROR(error, "Type size");
    ASSERTMSG(size == m_nBytesPerBody, "MPI type has not the same byte size");

    m_writebuffer.reserve(4000*((LayoutConfigType::LayoutType::NDOFqObj + LayoutConfigType::LayoutType::NDOFuObj)*sizeof(double)
                                        +1*sizeof(RigidBodyIdType)) + 1*sizeof(double) ); // t + 4000*(id,q,u)


}



template <typename TDynamicsSystemType>
MultiBodySimFileMPI<TDynamicsSystemType>::~MultiBodySimFileMPI(){
    int error = MPI_Type_free(&m_bodyStateTypeMPI);
    ASSERTMPIERROR(error, "Type free");
}

template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::close(){
     int err = MPI_File_close(&m_file_handle);
     ASSERTMPIERROR(err, "File close");
}


template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::write(double time, const RigidBodyContainer<RigidBodyType> & bodyList){

    //writeBySharedPtr(time, bodyList);
    writeByOffsets(time, bodyList);
    //writeByOffsets2(time, bodyList);
}

template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::writeBySharedPtr(double time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList)
{
    int err;
    m_writebuffer.clear();

    boost::iostreams::back_insert_device<std::vector<char> >ins(m_writebuffer); // is initialized first
    boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > stream(ins);  //is initialized second
    boost::archive::binary_oarchive  oa( stream,boost::archive::no_codecvt | boost::archive::no_header); // is initialized third

    for(auto it = bodyList.beginOrdered(); it!= bodyList.endOrdered();it++){

        oa << (*it)->m_id;
        serializeEigen(oa, (*it)->get_q());
        serializeEigen(oa, (*it)->get_u());
    }
    //Necessary to make stream flush
    stream.flush();


    ASSERTMSG( (bodyList.size() == 0 && m_writebuffer.size() == 0 )
                   || (bodyList.size() != 0 && m_writebuffer.size() != 0 ) , "m_writebuffer.size()" << m_writebuffer.size() );


    //bytes need to be a multiple of the bytes for one body state
    ASSERTMSG(m_writebuffer.size() % ( m_nBytesPerBody ) == 0, m_writebuffer.size() << " bytes not a multiple of " << m_nBytesPerBody << " bytes");

    std::vector<int> nbodiesPerProc(m_processes);
    unsigned int nBodies = bodyList.size();
    MPI_Status s;

    if(m_rank == 0 ){
        //write time
        err = MPI_File_write_shared(m_file_handle,&time,1,MPI_DOUBLE,&s);
        ASSERTMPIERROR(err, "write time");
    }
    // Use shared file pointer to write ordered stuff (collective, rank=0 will start
    err = MPI_File_write_ordered(m_file_handle,const_cast<void*>((const void*)&m_writebuffer[0]),nBodies,m_bodyStateTypeMPI,&s);
    ASSERTMPIERROR(err, "write states");

}

template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::writeByOffsets(double time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList)
{

    unsigned int nBodies = bodyList.size();
    MPI_Status s;

    //Next State offset
    MPI_Offset nextState;
    MPI_File_get_position(m_file_handle,&nextState);
    nextState += m_nBytesPerState;

    m_writebuffer.clear();
    if(nBodies != 0){
        boost::iostreams::back_insert_device<std::vector<char> >ins(m_writebuffer); // is initialized first
        boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > stream(ins);  //is initialized second
        boost::archive::binary_oarchive  oa( stream,boost::archive::no_codecvt | boost::archive::no_header); // is initialized third

        for(auto it = bodyList.beginOrdered(); it!= bodyList.endOrdered();it++){
            ASSERTMSG((*it)->m_id != RigidBodyIdType(0x0)," ID zero! at time: " << time);
            oa << (*it)->m_id;
            serializeEigen(oa, (*it)->get_q());
            serializeEigen(oa, (*it)->get_u());
        }
        //Necessary to make stream flush
        stream.flush();

        ASSERTMSG( (bodyList.size() == 0 && m_writebuffer.size() == 0 )
                       || (bodyList.size() != 0 && m_writebuffer.size() != 0 ) , "m_writebuffer.size()" << m_writebuffer.size() );
        //bytes need to be a multiple of the bytes for one body state
        ASSERTMSG(( m_writebuffer.size())  % ( m_nBytesPerBody ) == 0, ( m_writebuffer.size()) << " bytes not a multiple of " << m_nBytesPerBody << " bytes");
    }

    std::vector<int> nbodiesPerProc(m_processes);
    int err = MPI_Allgather(&nBodies,1,MPI_INT,&nbodiesPerProc[0] , 1, MPI_INT, m_comm);
    ASSERTMPIERROR(err, "gather");
    //Calculate our offset
    unsigned int bodyOffset = 0;
    MPI_Offset offsetMPI = 0;
    // All calculate offset [begin, begin + rank)
    ASSERTMSG( std::accumulate(nbodiesPerProc.begin(),nbodiesPerProc.end(),0) == m_nSimBodies, " accumulation not the same");
    if(m_rank != 0 ){
        bodyOffset = std::accumulate(nbodiesPerProc.begin(),nbodiesPerProc.begin() + m_rank , 0);
        offsetMPI = bodyOffset*m_nBytesPerBody;
    }

    // Find first process which has bodies (this one writes the time!)
    unsigned int rankWriteTime = 0;
    ASSERTMSG(rankWriteTime < m_processes," rankWriteTime: " << rankWriteTime);

    //Calculate offset

    if(m_rank > rankWriteTime){
        offsetMPI += sizeof(double); // for time
    }

    //each sets the offset
    err = MPI_File_seek(m_file_handle, offsetMPI, MPI_SEEK_CUR );
    ASSERTMPIERROR(err, " set offset");
    // Use the offsets to write collectively
    if(m_rank == rankWriteTime){
        err = MPI_File_write(m_file_handle,&time,1 * sizeof(double),MPI_BYTE,&s);
    }
    ASSERTMPIERROR(err, " write states");
    err = MPI_File_write_all(m_file_handle,const_cast<void*>((const void*)(&m_writebuffer[0])),nBodies,m_bodyStateTypeMPI,&s);
    ASSERTMPIERROR(err, " seek new state");

    // move individual file pointer to next begining!
    err = MPI_File_seek(m_file_handle, nextState, MPI_SEEK_SET );
    ASSERTMPIERROR(err, " seek new state");
}

template <typename TDynamicsSystemType>
void MultiBodySimFileMPI<TDynamicsSystemType>::writeByOffsets2(double time, const typename DynamicsSystemType::RigidBodySimContainerType & bodyList)
{

    unsigned int nBodies = bodyList.size();
    MPI_Status s;

    //Next State offset
    MPI_Offset nextState;
    MPI_File_get_position(m_file_handle,&nextState);
    nextState += m_nBytesPerState;

    m_writebuffer.clear();
    if(nBodies != 0){
        boost::iostreams::back_insert_device<std::vector<char> >ins(m_writebuffer); // is initialized first
        boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > stream(ins);  //is initialized second
        boost::archive::binary_oarchive  oa( stream,boost::archive::no_codecvt | boost::archive::no_header); // is initialized third

        for(auto it = bodyList.beginOrdered(); it!= bodyList.endOrdered();it++){
            ASSERTMSG((*it)->m_id != RigidBodyIdType(0x0)," ID zero! at time: " << time);
            oa << (*it)->m_id;
            serializeEigen(oa, (*it)->get_q());
            serializeEigen(oa, (*it)->get_u());
        }
        //Necessary to make stream flush
        stream.flush();

        ASSERTMSG( (bodyList.size() == 0 && m_writebuffer.size() == 0 )
                       || (bodyList.size() != 0 && m_writebuffer.size() != 0 ) , "m_writebuffer.size()" << m_writebuffer.size() );
        //bytes need to be a multiple of the bytes for one body state
        ASSERTMSG(( m_writebuffer.size())  % ( m_nBytesPerBody ) == 0, ( m_writebuffer.size()) << " bytes not a multiple of " << m_nBytesPerBody << " bytes");
    }

    std::vector<int> nbodiesPerProc(m_processes);
    int err = MPI_Allgather(&nBodies,1,MPI_INT,&nbodiesPerProc[0] , 1, MPI_INT, m_comm);
    ASSERTMPIERROR(err, "gather");
    //Calculate our offset
    unsigned int bodyOffset = 0;
    MPI_Offset offsetMPI = 0;
    // All calculate offset
    ASSERTMSG( std::accumulate(nbodiesPerProc.begin(),nbodiesPerProc.end(),0) == m_nSimBodies, " accumulation not the same");
    if(m_rank != 0 ){
        bodyOffset = std::accumulate(nbodiesPerProc.begin(),nbodiesPerProc.begin() + m_rank, 0);
        //Calculate offset
        offsetMPI = bodyOffset*m_nBytesPerBody;
    }


    unsigned int rankWriteTime = 0;


    offsetMPI += sizeof(double); // for time


    // Use the offsets to write collectively
    if(m_rank == rankWriteTime){
        err = MPI_File_write_at(m_file_handle,0,&time,1 * sizeof(double),MPI_BYTE,&s);
    }
    ASSERTMPIERROR(err, " write states");
    std::cout << "Rank: " << m_rank << " Size of buffer: " << m_writebuffer.size() << " Offset: " << bodyOffset<< std::endl;
    err = MPI_File_write_at_all(m_file_handle,offsetMPI,const_cast<void*>((const void*)(&m_writebuffer[0])),nBodies,m_bodyStateTypeMPI,&s);
    ASSERTMPIERROR(err, " seek new state");
    MPI_File_sync(m_file_handle);
    // move individual file pointer to next begining!
    //err = MPI_File_seek(m_file_handle, nextState, MPI_SEEK_SET );
//    ASSERTMPIERROR(err, " seek new state");
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
    m_nBytesPerState = nSimBodies*(m_nBytesPerBody) + 1*sizeof(double);
    m_nSimBodies = nSimBodies;
}

template <typename TDynamicsSystemType>
bool MultiBodySimFileMPI<TDynamicsSystemType>::openWrite(MPI_Comm comm, const boost::filesystem::path &file_path, const unsigned int nSimBodies, bool truncate) {

    // Duplicate the communicator, this file Io does not use the same communicator as all other stuff!
    MPI_Comm_dup(comm,&m_comm);

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
    // for no use, as the time is a double and should be a multipl. of element of etype size
    //int err = MPI_File_set_view(m_file_handle, (MPI_Offset)m_headerLength, m_bodyStateTypeMPI,m_bodyStateTypeMPI,type,MPI_INFO_NULL);

    int err = MPI_File_set_view(m_file_handle, (MPI_Offset)m_headerLength, MPI_BYTE,MPI_BYTE,type,MPI_INFO_NULL);

    if(!mpiSucceded(err)){
        return false;
    }

    return true;
}


#endif

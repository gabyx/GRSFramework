#ifndef MultiBodySimFileMPI_hpp
#define MultiBodySimFileMPI_hpp


#include <type_traits>
#include <vector>
#include <sstream>
#include <fstream>

#include <boost/filesystem.hpp>

#include "TypeDefs.hpp"
#include "StaticAssert.hpp"

#include DynamicsSystem_INCLUDE_FILE


#include "CommonFunctions.hpp"
#include "MPISerializationHelpersEigen.hpp"
#include "MultiBodySimFileIOHelpers.hpp"

#define SIM_FILE_MPI_SIGNATURE_LENGTH 4
#define SIM_FILE_MPI_SIGNATURE {'M','B','S','F'}

#define SIM_FILE_MPI_VERSION 1

#define SIM_FILE_MPI_EXTENSION ".simmpi"

class MultiBodySimFileMPI {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    MultiBodySimFileMPI(unsigned int nDOFqBody, unsigned int nDOFuBody);
    ~MultiBodySimFileMPI();

    bool openWrite(MPI_Comm comm,  const boost::filesystem::path & file_path,   const unsigned int nSimBodies,  bool truncate = true);
    void close();

    inline void write(double time, const std::vector<char> & bytes, unsigned int nBodies);

    inline void write(double time, const RigidBodyContainer & bodyList) {
        //writeBySharedPtr(time, bodyList);
        writeByOffsets(time, bodyList);
        //writeByOffsets2(time, bodyList);
    }


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

    unsigned int m_nDOFuBody, m_nDOFqBody;


    const  std::streamoff m_nBytesPerQBody ;
    const  std::streamoff m_nBytesPerUBody ;

    static const unsigned short m_additionalBytesType = 1;
    static constexpr std::streamoff setAdditionalBytes(){
        return (m_additionalBytesType==1) ? 1*sizeof(unsigned short) : 0 ;
    }
    static const  std::streamoff m_nAdditionalBytesPerBody;

    const  std::streamoff m_nBytesPerBody; ///< id,q,u + m_nAdditionalBytesPerBody

    static const  std::streamoff m_headerLength = (4*sizeof(unsigned int) + SIM_FILE_MPI_SIGNATURE_LENGTH*sizeof(char)); ///< nBodies, NDOFq, NDOFu, additionalBytesType (0=nothing, 1 = + process rank, etc.)

    MultiBodySimFileMPI & operator =(const MultiBodySimFileMPI & file);

    std::vector<char> m_writebuffer;

    std::stringstream m_errorString;

    bool mpiSucceded(int err) {
        m_errorString.str("");
        if(err != MPI_SUCCESS) {
            char * string;
            int length;
            MPI_Error_string( err , string, &length );
            m_errorString << string;
            return false;
        }
        return true;
    }

};

/** Function template to add the spcific bytes */
template<unsigned int type> struct AddBytes;

template<>
struct AddBytes<1>{
    template<typename Archive, typename TRigidBody >
    static void write(Archive & oa, TRigidBody *body) {
        oa << body->m_pBodyInfo->m_ownerRank; // write owner rank
    }
};

template<>
struct AddBytes<0>{
    template<typename Archive, typename TRigidBody >
    static void write(Archive  & oa, TRigidBody *body) {
        return;
    }
};


#endif

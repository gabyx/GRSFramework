#ifndef MPICommunicationFunctions_hpp
#define MPICommunicationFunctions_hpp

#include <mpi.h>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>

#include "AssertionDebug.hpp"
#include "AssertionDebugMPI.hpp"
#include "TypeDefs.hpp"

#include "MPIInformation.hpp"



namespace MPILayer {


/**
*    Important struct to define all MPI message tags used in this framework!
*/
class MPIMessageTags {
public:

    template<typename T>
    static unsigned int getTag() {
        getTag(type<T>());
    };

private:
    template<typename T> struct type {};

    static unsigned int getTag(type<std::string>) {
        return STDSTRING;
    }

    enum {
        RIGIDBODY_NUMBER_CHECK,
        RIGIDBODY_UPDATE_MESSAGE,
        RIGIDBODY_MESSAGE,
        CONTACT_UPDATE_MESSAGE,
        STDSTRING,
        GENERIC_STRING_MESSAGE,
        CURRENT_SIMFOLDER_MESSAGE
    };
};


/**
* Composer which serilaizes one message: take care sending bytes and receiving them need to make sure that the same endianess is used in the network!
*/
class MessageBinarySerializer {
public:

    MessageBinarySerializer(std::size_t reserve_bytes) {
        m_buffer.reserve(reserve_bytes);
    }

    void clear() { // Clears the buffer
        m_buffer.clear();
    }

    /** Serialization of one object */
    template<typename T>
    MessageBinarySerializer & operator<<(const T & t) {
        m_buffer.clear(); //Clear serializable string, no allocation if we push less or equal as much into the string next time!
        boost::iostreams::back_insert_device<std::vector<char> > inserter(m_buffer);
        boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > s(inserter);
        boost::archive::binary_oarchive oa(s);
        oa << t;
        s.flush();

        return *this;
    };

    /** Deserialization */
    template<typename T>
    MessageBinarySerializer & operator>>(T & t) {
        boost::iostreams::basic_array_source<char> device(data(), size());
        boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
        boost::archive::binary_iarchive ia(s);
        ia >> t;

        return *this;
    };

    MPI_Datatype getMPIDataType() {
        return MPI_CHAR;
    }
    const  char * data() {
        return &m_buffer[0];
    }

    std::size_t size() {
        return m_buffer.size();
    }

    void resize(std::size_t bytes) {
        m_buffer.resize(bytes);
    }

    void reserve(std::size_t bytes) {
        m_buffer.reserve(bytes);
    }

private:
    std::vector<char> m_buffer;
};


template<typename TDynamicsSystem>
class ProcessCommunicator {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef MPILayer::ProcessInformation<DynamicsSystemType> ProcessInfoType;
    typedef typename ProcessInfoType::RankIdType RankIdType;

    ProcessCommunicator():
        m_pProcessInfo(new ProcessInfoType()),
        m_binary_message(1024*1024)
    {};

    template<typename T>
    void sendBroadcast(const T & t, MPI_Comm comm);

    template<typename T>
    void receiveBroadcast(T & t, RankIdType rank, MPI_Comm comm);

    void sendBroadcast(const std::string & t, MPI_Comm comm);

    void receiveBroadcast(std::string & t, RankIdType rank, MPI_Comm comm);

    template<typename T>
    void sendMessageToRank(const T & t, RankIdType rank, MPI_Comm comm);

    boost::shared_ptr<ProcessInfoType> getProcInfo() {
        return m_pProcessInfo;
    }

private:

    boost::shared_ptr<ProcessInfoType> m_pProcessInfo;

    MessageBinarySerializer m_binary_message; // 1 MB serialization buffer
};

template<typename TDynamicsSystem>
template<typename T>
void ProcessCommunicator<TDynamicsSystem>::sendBroadcast(const T & t, MPI_Comm comm) {

    m_binary_message.clear();

    m_binary_message << t ;

    int size = m_binary_message.size();

    int error = MPI_Bcast(&(size), 1 , MPI_INT, m_pProcessInfo->getRank(), comm); // First send size, because we cannot probe on the other side!! Collective Communication
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    MPI_Bcast(const_cast<char*>(m_binary_message.data()), m_binary_message.size(), m_binary_message.getMPIDataType(), m_pProcessInfo->getRank(), comm);
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
};

template<typename TDynamicsSystem>
template<typename T>
void ProcessCommunicator<TDynamicsSystem>::receiveBroadcast(T & t, RankIdType rank, MPI_Comm comm) {

    int message_length;

    int error = MPI_Bcast(&message_length, 1 , MPI_INT, rank, comm);
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    m_binary_message.resize(message_length);

    error = MPI_Bcast(const_cast<char*>(m_binary_message.data()), m_binary_message.size(), m_binary_message.getMPIDataType(), rank, comm);
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    m_binary_message >> t;
};

template<typename TDynamicsSystem>
void ProcessCommunicator<TDynamicsSystem>::sendBroadcast(const std::string & t, MPI_Comm comm) {
    int size = t.size();
    int error = MPI_Bcast(&(size), 1 , MPI_INT, m_pProcessInfo->getRank(), comm); // First send size, because we cannot probe on the other side!! Collective Communication
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    MPI_Bcast(const_cast<char*>(t.data()), t.size(), MPI_CHAR, m_pProcessInfo->getRank(), comm);
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
};

template<typename TDynamicsSystem>
void ProcessCommunicator<TDynamicsSystem>::receiveBroadcast(std::string & t, RankIdType rank, MPI_Comm comm) {

    int message_length;

    int error = MPI_Bcast(&message_length, 1 , MPI_INT, rank, comm);
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    t.resize(message_length);

    error = MPI_Bcast(t.data(), t.size() , MPI_CHAR, rank, comm);
    ASSERTMPIERROR(error,"MPISendBroadcast failed!");
    m_binary_message >> t;
};

template<typename TDynamicsSystem>
template<typename T>
void ProcessCommunicator<TDynamicsSystem>::sendMessageToRank(const T & t, RankIdType rank, MPI_Comm comm){


};

}; // MPILayer

#endif

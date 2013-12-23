#ifndef MPICommunicationFunctions_hpp
#define MPICommunicationFunctions_hpp

#include <mpi.h>

#include <boost/tuple/tuple.hpp>

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/version.hpp>

#include "AssertionDebug.hpp"
#include "StaticAssert.hpp"
#include "TypeDefs.hpp"

#include "MPIInformation.hpp"


namespace MPILayer {


/**
*    Important struct to define all MPI message tags used in this framework!
*/

class MPIMessageTag {
    public:
        enum class Type: unsigned int {
            GENERICMESSAGE = 1 << 0,
            STDSTRING =      1 << 1,
            BODY_MESSAGE =   1 << 2,
            EXTERNALCONTACTS_MESSAGE = 1 << 3,
            SPLITBODYFACTOR_MESSAGE  = 1 << 4,
            SPLITBODYUPDATE_MESSAGE  = 1 << 5,
            SPLITBODYSOLUTION_MESSAGE = 1<< 6
        };

        MPIMessageTag( Type t): m_t(t){};

        int getInt(){return static_cast<int>(m_t);}

    private:
        Type m_t;
};


/**
* Composer which serilaizes one message: take care sending bytes and receiving them need to make sure that the same endianess is used in the network!
*/
class MessageBinarySerializer {
public:

    MessageBinarySerializer(std::size_t reserve_bytes) {
        m_buffer.reserve(reserve_bytes);
    }

    // Move copy constructor
    MessageBinarySerializer( MessageBinarySerializer && ref): m_buffer(std::move(ref.m_buffer)) {}

    // Move assignment
    MessageBinarySerializer & operator=(MessageBinarySerializer && ref) {
        m_buffer = std::move(ref.m_buffer);
        return *this;
    }

    void clear() { // Clears the buffer, but does not affect std::vector.capacity!
        m_buffer.clear();
    }

    /** Serialization of one object */
    template<typename T>
    MessageBinarySerializer & operator<<(const T & t) {
        m_buffer.clear(); //Clear serializable string, no allocation if we push less or equal as much into the string next time!
        boost::iostreams::back_insert_device<std::vector<char> > inserter(m_buffer);
        boost::iostreams::stream< boost::iostreams::back_insert_device<std::vector<char> > > s(inserter);
        boost::archive::binary_oarchive oa(s, boost::archive::no_codecvt | boost::archive::no_header);
        oa << t;
        s.flush();

        return *this;
    };

    /** Deserialization */
    template<typename T>
    MessageBinarySerializer & operator>>(T & t) {
        boost::iostreams::basic_array_source<char> device(data(), size());
        boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s(device);
        boost::archive::binary_iarchive ia(s,  boost::archive::no_codecvt | boost::archive::no_header);
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


class ProcessCommunicator {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    typedef MPILayer::ProcessInformation ProcessInfoType;
    typedef typename ProcessInfoType::RankIdType RankIdType;

    ProcessCommunicator():
        m_pProcessInfo(new ProcessInfoType()),
        m_binary_message(1024*1024)
    {
        if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
            m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        } else {
            ERRORMSG("SimulationLog does not yet exist? Did you create it?")
        }
    };

    template<typename T>
    void sendBroadcast(const T & t, MPI_Comm comm = MPI_COMM_WORLD){
        m_binary_message.clear();
        m_binary_message << t ;
        int size = m_binary_message.size();
        int error = MPI_Bcast(&(size), 1 , MPI_INT, m_pProcessInfo->getRank(), comm); // First send size, because we cannot probe on the other side!! Collective Communication
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcastT1 failed!");
        MPI_Bcast(const_cast<char*>(m_binary_message.data()), m_binary_message.size(), m_binary_message.getMPIDataType(), m_pProcessInfo->getRank(), comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcastT2 failed!");
    };

    template<typename T>
    void receiveBroadcast(T & t, RankIdType rank, MPI_Comm comm = MPI_COMM_WORLD) {
        int message_length;

        int error = MPI_Bcast(&message_length, 1 , MPI_INT, rank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcastT1 failed!");
        m_binary_message.resize(message_length);

        error = MPI_Bcast(const_cast<char*>(m_binary_message.data()), m_binary_message.size(), m_binary_message.getMPIDataType(), rank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcastT2 failed!");
        m_binary_message >> t;
    };

    void sendBroadcast(const std::string & t, MPI_Comm comm = MPI_COMM_WORLD){
        int size = t.size();
        int error = MPI_Bcast(&(size), 1 , MPI_INT, m_pProcessInfo->getRank(), comm); // First send size, because we cannot probe on the other side!! Collective Communication
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcast1 failed!");
        MPI_Bcast(const_cast<char*>(t.data()), t.size(), MPI_CHAR, m_pProcessInfo->getRank(), comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcast2 failed!");
    };

    void receiveBroadcast(std::string & t, RankIdType rank, MPI_Comm comm = MPI_COMM_WORLD) {
        int message_length;

        int error = MPI_Bcast(&message_length, 1 , MPI_INT, rank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcast1 failed!");
        t.resize(message_length);

        error = MPI_Bcast(const_cast<char*>(t.data()), t.size() , MPI_CHAR, rank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcast2 failed!");
        m_binary_message >> t;
    };

    template<typename T>
    void sendMessageToRank(const T & t, RankIdType rank, MPIMessageTag tag, MPI_Comm comm = MPI_COMM_WORLD){
        auto it = m_sendMessageBuffers.find(rank);
        ASSERTMSG(it != m_sendMessageBuffers.end(),"No buffer for this rank!, Did you call initializeBuffers" );
        // clear the buffer

        MessageBinarySerializer & message = std::get<0>(it->second);
        MPI_Request* req = std::get<1>(it->second); // Get copy of the pointer (double pointer)

        message.clear();
        // Serialize the message into the buffer
        message << t ;
        // Nonblocking Send (we keep the buffer message till the messages have been sent!)
        // If MPI uses buffering, these message might have been sent before the receive has been posted
        // if there is no buffering in MPI we post the sends (leave the buffer existing)
        int error = MPI_Isend( const_cast<char*>(message.data()) , message.size(), message.getMPIDataType(),
                              rank, tag.getInt(), comm, req );
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendMessageToRank failed!");

        // IMPORTANT! Always call a waitForAllSends() after this call!!!
    }

    void waitForAllSends(){

        LOGPC(m_pSimulationLog,  "--->\t\t Waiting for all sends to complete ..." << std::endl;)
        int error = MPI_Waitall(m_sendRequests.size(), &m_sendRequests[0],&m_sendStatuses[0]);

        ASSERTMPIERROR(error, "ProcessCommunicator:: waiting for sends failed")
    }

    template<typename T, typename List>
    void receiveMessageFromRanks(T & t,const List & ranks, MPIMessageTag tag,  MPI_Comm comm = MPI_COMM_WORLD ){
        STATIC_ASSERT( (std::is_same<RankIdType, typename List::value_type>::value) );

        if(ranks.size() == 0){return;};

        typename List::const_iterator ranksIt;
        std::vector<bool> receivedRanks(ranks.size(),false);

        // has an entry if a message has already been received for this rank.
        MPI_Status status;

        int received_messages = 0;
        int flag, i;
        LOGPC(m_pSimulationLog,  "--->\t\t Receiving message from neighbours (spin loop)..." << std::endl;)
        while( received_messages !=  ranks.size()){
            auto itBool = receivedRanks.begin();
            for(ranksIt = ranks.begin(); ranksIt !=  ranks.end(); ++ranksIt , ++itBool){

                if(*itBool == true){
                    //Received this rank already! Skip
                    continue;
                }

                // Probe for a message!
                //LOGPC(m_pSimulationLog,  "--->\t\t Waiting for message from neighbour: " << *ranksIt << std::endl;)

                int error = MPI_Iprobe( (int)(*ranksIt), tag.getInt(), comm, &flag, &status); // Non blocking test if message is there!
                ASSERTMPIERROR(error,"ProcessCommunicator:: receiveMessageFromRanks1 failed for rank " << *ranksIt )

                if(flag){ // We received a message

                    RankIdType recv_rank = status.MPI_SOURCE;

                    //Receive the message for this rank!
                    int message_size;
                    MPI_Get_count(&status,m_binary_message.getMPIDataType(),&message_size);
                    m_binary_message.resize(message_size);

                    error = MPI_Recv( const_cast<char*>(m_binary_message.data()),
                                          m_binary_message.size(),
                                          m_binary_message.getMPIDataType(),
                                          status.MPI_SOURCE, status.MPI_TAG, comm, &status
                                         );
                    LOGPC(m_pSimulationLog, "--->\t\t Received message from rank: " << recv_rank << " [" << message_size << "b]" << std::endl;)
                    ASSERTMPIERROR(error, "ProcessCommunicator:: receiveMessageFromRanks2 failed for rank "<< *ranksIt)

                    // Deserialize
                    t.setRank(recv_rank); // Set the rank
                    m_binary_message >> t;

                      // Set the bool to true for this index
                    *itBool = true;
                    received_messages++; // count up
                }
            }
        }

    };

    boost::shared_ptr<ProcessInfoType> getProcInfo() {
        return m_pProcessInfo;
    }

    /** May be called after a process topo has been made in the process info*/
    void initializeBuffers() {
        if( ! m_pProcessInfo->getProcTopo() ){
            ERRORMSG("initializeBuffers:: ProcessTopology is not created!")
        }else{

            typename ProcessInfoType::ProcessTopologyType::NeighbourRanksListType ranks = m_pProcessInfo->getProcTopo()->getNeighbourRanks();

            // Reserve space , if not the vector might reallocate itself!!!
            m_sendRequests.reserve(ranks.size());
            for(auto it = ranks.begin(); it != ranks.end(); it++){
                //this takes advantage of move semantics in MessageBinarySerializer
                m_sendStatuses.push_back(MPI_Status());
                m_sendRequests.push_back(NULL);
                m_sendMessageBuffers.insert(std::make_pair(
                                                        *it,
                                                        SendTupleType(MessageBinarySerializer(1024*1024),
                                                                      &m_sendRequests.back()
                                                                      ))
                                                        );

            }

        }
    }

private:

    Logging::Log *  m_pSimulationLog;

    boost::shared_ptr<ProcessInfoType> m_pProcessInfo;

    // Standart binary message for standart communication
    MessageBinarySerializer m_binary_message; // 1 MB serialization buffer


    typedef std::tuple< MessageBinarySerializer, MPI_Request*> SendTupleType;
    typedef std::unordered_map<RankIdType, SendTupleType> BufferMapType;
    BufferMapType m_sendMessageBuffers;
    std::vector<MPI_Request>                      m_sendRequests; ///< these are the requests, these are pointers!
    std::vector<MPI_Status>                       m_sendStatuses;
};

}; // MPILayer

#endif

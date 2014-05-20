#ifndef MPICommunicationFunctions_hpp
#define MPICommunicationFunctions_hpp

#include <type_traits>

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
#include "MPIMessageTag.hpp"
#include "MPICommunicatorId.hpp"

namespace MPILayer {


/**
*    Important struct to define all MPI message tags used in this framework!
*/


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


class ProcessCommunicator : public ProcessInformation{
public:

    //DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    typedef ProcessInformation ProcessInfoType;
    typedef ProcessInfoType BaseType;
    typedef typename ProcessInfoType::RankIdType RankIdType;

    /**
    * General communication is done with the communicator comm, but also with other communicators is possible
    */
    ProcessCommunicator(MPI_Comm comm):
        ProcessInformation(comm),
        m_binary_message(1024*1024)
    {
        if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")) {
            m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        } else {
            ERRORMSG("SimulationLog does not yet exist? Did you create it?")
        }

    };


    ~ProcessCommunicator(){
        for(auto it = m_communicators.begin(); it!= m_communicators.end(); it++){
            auto error =  MPI_Comm_free( &(it->second) );
            ASSERTMPIERROR(error, "MPI_Comm_free failed in rank: " << this->getRank());
        }
    }


    template<typename T>
    void sendBroadcast(const T & t, RankIdType rootRank, MPI_Comm comm){
        m_binary_message.clear();
        m_binary_message << t ;
        int size = m_binary_message.size();
        int error = MPI_Bcast(&(size), 1 , MPI_INT, rootRank, comm); // First send size, because we cannot probe on the other side!! Collective Communication
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcastT1 failed!");
        MPI_Bcast(const_cast<char*>(m_binary_message.data()),
                  m_binary_message.size(),
                  m_binary_message.getMPIDataType(),
                  rootRank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcastT2 failed!");
    };
    template<typename T>
    inline void sendBroadcast(T & t){sendBroadcast(t,this->m_rank, this->m_comm);}

    template<typename T>
    void receiveBroadcast(T & t, RankIdType rootRank, MPI_Comm comm) {
        int message_length;

        int error = MPI_Bcast(&message_length, 1 , MPI_INT, rootRank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcastT1 failed!");
        m_binary_message.resize(message_length);

        error = MPI_Bcast(const_cast<char*>(m_binary_message.data()), m_binary_message.size(), m_binary_message.getMPIDataType(), rootRank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcastT2 failed!");
        m_binary_message >> t;
    };
    template<typename T>
    inline void receiveBroadcast(T & t, RankIdType rootRank){receiveBroadcast(t,rootRank,this->m_comm);}


    void sendBroadcast(const std::string & t, RankIdType rootRank,  MPI_Comm comm){
        int size = t.size();
        int error = MPI_Bcast(&(size), 1 , MPI_INT, rootRank, comm); // First send size, because we cannot probe on the other side!! Collective Communication
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcast1 failed!");
        MPI_Bcast(const_cast<char*>(t.data()), t.size(), MPI_CHAR, rootRank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendBroadcast2 failed!");
    };
    inline void sendBroadcast(const std::string & t){sendBroadcast(t,this->m_rank, this->m_comm);}


    void receiveBroadcast(std::string & t, RankIdType rootRank, MPI_Comm comm) {
        int message_length;

        int error = MPI_Bcast(&message_length, 1 , MPI_INT, rootRank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcast1 failed!");
        t.resize(message_length);

        error = MPI_Bcast(const_cast<char*>(t.data()), t.size() , MPI_CHAR, rootRank, comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: receiveBroadcast2 failed!");
        m_binary_message >> t;
    };
    inline void receiveBroadcast(std::string & t, RankIdType rootRank){receiveBroadcast(t,rootRank,m_comm);}


    template<typename T>
    void allGather(T value, std::vector<T> & gatheredValues, MPI_Comm comm){

        // Assert on std::vector<bool> (bitwise implementation which fails for c array style)
        STATIC_ASSERT2((!std::is_same<T,bool>::value), "WORKS ONLY FOR non bool")
        // GCC No support STATIC_ASSERT2(std::is_trivially_copyable<T>::value, "WORKS ONLY FOR TRIVIALLY COPIABLE TYPES")
        STATIC_ASSERT2(std::is_trivial<T>::value, "WORKS ONLY FOR TRIVIALLY COPIABLE TYPES")

        int size;
        MPI_Comm_size(comm,&size);
        gatheredValues.resize(size);

        char * p = static_cast<char*>(gatheredValues.data());

        int error = MPI_Allgather(&value,sizeof(T),MPI_BYTE, p , sizeof(T), MPI_BYTE, comm);
        ASSERTMPIERROR(error, "ProcessCommunicator:: allGather failed");
    }
    template<typename T>
    void allGather(T value, std::vector<T> & gatheredValues){ allGather(value,gatheredValues,m_comm); }
    template<typename T>
    inline void allGather(T value, std::vector<T> & gatheredValues, MPICommunicatorId id){
        auto it = m_communicators.find(static_cast<unsigned int>(id));
        ASSERTMSG(it!=m_communicators.end(),"This communicator does not exist in the set in rank: " << this->getRank())
        allGather(value,gatheredValues,it->second);
    }




    /** Buffered Neighbour Send Begin ===============================================================================*/
    template<typename T>
    void sendMessageToNeighbourRank(const T & t, RankIdType rank, MPIMessageTag tag, MPI_Comm comm ){
        auto it = m_sendMessageBuffers.find(rank);
        ASSERTMSG(it != m_sendMessageBuffers.end(),"No buffer for this rank!, Did you call initializeNeighbourBuffers" );
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
                              rank, static_cast<int>(tag), comm, req );
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendMessageToNeighbourRank failed!");

        // IMPORTANT! Always call a waitForAllNeighbourSends() after this call!!!
    }
    template<typename T>
    void sendMessageToNeighbourRank(const T & t, RankIdType rank, MPIMessageTag tag ){ sendMessageToNeighbourRank(t,rank,tag,this->m_comm);}

    void waitForAllNeighbourSends(){

        LOGPC(m_pSimulationLog,  "--->\t\t Waiting for all sends to complete ..." << std::endl;)
        int error = MPI_Waitall(m_sendRequests.size(), &m_sendRequests[0],&m_sendStatuses[0]);

        ASSERTMPIERROR(error, "ProcessCommunicator:: waiting for sends failed")
    }

     /** May be called after a process topo has been made in the process info*/
    void initializeNeighbourBuffers() {
        if( ! this->getProcTopo() ){
            ERRORMSG("initializeNeighbourBuffers:: ProcessTopology is not created!")
        }else{

            typename ProcessInfoType::ProcessTopologyType::NeighbourRanksListType ranks = this->getProcTopo()->getNeighbourRanks();
            m_sendStatuses.clear();
            m_sendRequests.clear();
            m_sendMessageBuffers.clear();

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

    /** Buffered Neighbour Send Finished ===============================================================================*/


    template<typename T>
    void sendMessageToRank(const T & t, RankIdType rank, MPIMessageTag tag, MPI_Comm comm ){
        m_binary_message.clear();
        // Serialize the message into the buffer
        m_binary_message << t ;
        // Blocking Send
        // If MPI does not use buffering this function might hang if the receive are not posted correctly!
        // See unsafe MPI
        int error = MPI_Send( const_cast<char*>(m_binary_message.data()) , m_binary_message.size(), m_binary_message.getMPIDataType(),
                              rank, static_cast<int>(tag), comm);
        ASSERTMPIERROR(error,"ProcessCommunicator:: sendMessageToRank failed!");
    }

    template<typename T>
    inline void sendMessageToRank(const T & t, RankIdType rank, MPIMessageTag tag){
        sendMessageToRank(t,rank,tag,this->m_comm);
    }


    template<typename T, typename List>
    void receiveMessageFromRanks(T & t,const List & ranks, MPIMessageTag tag,  MPI_Comm comm  ){
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

                int error = MPI_Iprobe( (int)(*ranksIt), static_cast<int>(tag), comm, &flag, &status); // Non blocking test if message is there!
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
    template<typename T, typename List>
    void receiveMessageFromRanks(T & t, const List & ranks, MPIMessageTag tag ){ receiveMessageFromRanks(t,ranks,tag,this->m_comm);}


    /**
    * Inserts/Overwrites a new splitted communicator with id newCommId if color is not -1
    * No communicator freeing is done here!
    * Rank ordering is preserved from the old communicator
    */
    void generateCommunicatorSplitAndOverwrite(MPICommunicatorId newCommId, int groupColor){

        unsigned int id = static_cast<unsigned int>(newCommId);
        // groupColor < 0  is MPI_UNDEFINED
        if(groupColor<0){
            groupColor = MPI_UNDEFINED;
        }

        MPI_Comm comm;
        /// Collective call, all processes in comm need to call this function!
        auto error = MPI_Comm_split(m_comm,groupColor,this->m_rank,&comm);
        ASSERTMPIERROR(error, "MPI_Comm_split failed in rank: " << this->m_rank);

        ASSERTMSG(m_communicators.find(id) == m_communicators.end(), "This communicator is already in the set! This should not happend!")

        if(comm != MPI_COMM_NULL){
            m_communicators[id] = comm;
        }

    }

    /** Return a copy of the communicator with id commId*/
    using BaseType::getCommunicator;
    MPI_Comm getCommunicator( MPICommunicatorId commId ){
        auto it = m_communicators.find(static_cast<unsigned int>(commId));
        if(it!= m_communicators.end()){
            return it->second;
        }else{
            ERRORMSG("Communicator with id: " << static_cast<unsigned int>(commId) <<" does not exist!");
        }
    }

    void deleteAllCommunicators(){
        //Collective call to free all communicators, this
        // Take care, dead lock might happen if communicators are freed not according to a sorted global id!
        // our id is the MPICommunicatorId which are global, and m_communicator is sorted according to these ids
        for(auto it = m_communicators.begin(); it!= m_communicators.begin(); it++){
             auto error =  MPI_Comm_free( &(it->second) );
             ASSERTMPIERROR(error, "MPI_Comm_free failed in rank: " << this->m_rank)
        }
        m_communicators.clear();
    }

private:

    Logging::Log *  m_pSimulationLog;

    // Standart binary message for standart communication
    MessageBinarySerializer m_binary_message; // 1 MB serialization buffer

    /** These are buffers for the asynchronous send of the messages to neighbours to make it a safe MPI program (buffer problem) */
    typedef std::tuple< MessageBinarySerializer, MPI_Request*> SendTupleType;
    typedef std::unordered_map<RankIdType, SendTupleType> BufferMapType;
    BufferMapType m_sendMessageBuffers;
    std::vector<MPI_Request>                      m_sendRequests; ///< these are the requests, these are pointers!
    std::vector<MPI_Status>                       m_sendStatuses;

    /** These communicators are subject to change during communications, id's are chosen from MPICommunicatorId enumration
    *   Ids are global and thus on each process the same id corresponds to the same type of communicator
    *   Therefore freeing these communicators collectively in deleteAllCommunicators does not result in deadlock
    */
    std::map<unsigned int, MPI_Comm> m_communicators;
};

}; // MPILayer

#endif

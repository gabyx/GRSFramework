#ifndef NeighbourDataBodyCommunication_hpp
#define NeighbourDataBodyCommunication_hpp


#include <vector>
#include <list>
#include <type_traits>

#include "TypeDefs.hpp"

/**
* @brief This class is used in the NeighbourMap class as data structure for the general communication of bodies
*/
class NeighbourDataBodyCommunication {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES


    struct RemoteData{
        RemoteData(RigidBodyType * body):m_body(body){};
        RigidBodyType * const m_body;
    };
    struct LocalData{
        LocalData(RigidBodyType * body):m_body(body),m_commStatus(SEND_NOTIFICATION){};
        RigidBodyType * const m_body;
        enum {SEND_NOTIFICATION, SEND_UPDATE, SEND_REMOVE} m_commStatus;
    };

    typedef std::map<typename RigidBodyType::RigidBodyIdType, RemoteData > RemoteBodiesMapType;
    typedef typename RemoteBodiesMapType::iterator RemoteIterator;

    typedef std::map<typename RigidBodyType::RigidBodyIdType, LocalData > LocalBodiesMapType;
    typedef typename LocalBodiesMapType::iterator LocalIterator;


    NeighbourDataBodyCommunication(RankIdType neighbourRank): m_neighbourRank(neighbourRank){};
    ~NeighbourDataBodyCommunication(){
        m_remoteBodies.clear();
        m_localBodies.clear();
    }

    unsigned int sizeLocal(){
        return m_localBodies.size();
    }
    unsigned int sizeRemote(){
        return m_remoteBodies.size();
    }

    LocalIterator localBegin(){ return m_localBodies.begin(); }
    RemoteIterator remoteBegin(){ return m_remoteBodies.begin(); }
    LocalIterator localEnd(){ return m_localBodies.end(); }
    RemoteIterator remoteEnd(){ return m_remoteBodies.end(); }

    std::pair<LocalData*, bool> addLocalBodyData(RigidBodyType * body){

        auto resPair = m_localBodies.insert(
                       typename LocalBodiesMapType::value_type(body->m_id, LocalData(body) )
                    );
        return std::pair<LocalData *, bool>(&resPair.first->second, resPair.second);
    }


    std::pair<RemoteData*, bool> addRemoteBodyData(RigidBodyType * body){
        // <iterator,bool>
        auto resPair = m_remoteBodies.insert(
                    typename RemoteBodiesMapType::value_type(body->m_id, RemoteData(body) )
                    );

        return std::pair<RemoteData *, bool>(&resPair.first->second, resPair.second);
    }

    RemoteData * getRemoteBodyData(RigidBodyType * body){
        return  getRemoteBodyData(body->m_id);
    }

    RemoteData * getRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteBodies.find(id);
        if(it != m_remoteBodies.end()){
           return (&it->second);
        }else{
           //ASSERTMSG(false,"There is no RemoteData for body with id: " << id << "!")
           return NULL;
        }
    }

    LocalData * getLocalBodyData(RigidBodyType * body){
        return  getLocalBodyData(body->m_id);
    }

    LocalData * getLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localBodies.find(id);
        if(it != m_localBodies.end()){
           return (&it->second);
        }else{
           //ASSERTMSG(false,"There is no LocalData for body with id: " << id << "!")
           return NULL;
        }
    }

    bool deleteLocalBodyData(RigidBodyType * body){
        return deleteLocalBodyData(body->m_id);
    }
    bool deleteLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localBodies.find(id);
        if(it != m_localBodies.end()){
            m_localBodies.erase(it);
            return true;
        }
        return false;
    }

    bool deleteRemoteBodyData(RigidBodyType * body){
        return deleteRemoteBodyData(body->m_id);
    }

    bool deleteRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteBodies.find(id);
        if(it != m_remoteBodies.end()){
            m_remoteBodies.erase(it);
            return true;
        }
        return false;
    }

    void cleanUp();

private:
    RankIdType m_neighbourRank; // This is the rank to which this data structure belongs!

    /** Private because we do not want that the operator[] is called anywhere in these maps!
    * Not good behaviour if oeprator [] is used, as new element get inserted easily!
    */
    RemoteBodiesMapType m_remoteBodies; // All overlapping remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    LocalBodiesMapType m_localBodies; // All overlapping local bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)
};

#endif // NeighbourDataBodyCommunication_hpp

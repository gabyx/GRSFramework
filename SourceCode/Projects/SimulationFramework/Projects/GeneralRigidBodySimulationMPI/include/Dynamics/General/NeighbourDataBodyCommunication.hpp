#ifndef NeighbourDataBodyCommunication_hpp
#define NeighbourDataBodyCommunication_hpp


#include <vector>
#include <map>


#include "TypeDefs.hpp"

template<typename TLocalData, typename TRemoteData>
class NeighbourData{
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    typedef TLocalData LocalDataType;
    typedef TLocalData RemoteDataType;

    typedef std::unordered_map<typename RigidBodyType::RigidBodyIdType, RemoteDataType > RemoteBodiesMapType;
    typedef typename RemoteBodiesMapType::iterator RemoteIterator;

    typedef std::unordered_map<typename RigidBodyType::RigidBodyIdType, LocalDataType > LocalBodiesMapType;
    typedef typename LocalBodiesMapType::iterator LocalIterator;


    NeighbourData(const RankIdType & neighbourRank): m_neighbourRank(neighbourRank){};
    ~NeighbourData(){
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

    std::pair<LocalDataType*, bool> addLocalBodyData(RigidBodyType * body){

        auto resPair = m_localBodies.insert(
                       typename LocalBodiesMapType::value_type(body->m_id, LocalDataType(body) )
                    );
        return std::pair<LocalDataType *, bool>(&resPair.first->second, resPair.second);
    }


    std::pair<RemoteDataType*, bool> addRemoteBodyData(RigidBodyType * body){
        // <iterator,bool>
        auto resPair = m_remoteBodies.insert(
                    typename RemoteBodiesMapType::value_type(body->m_id, RemoteDataType(body) )
                    );

        return std::pair<RemoteDataType *, bool>(&resPair.first->second, resPair.second);
    }

    RemoteDataType * getRemoteBodyData(RigidBodyType * body){
        return  getRemoteBodyData(body->m_id);
    }

    RemoteDataType * getRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteBodies.find(id);
        if(it != m_remoteBodies.end()){
           return (&it->second);
        }else{
           //ASSERTMSG(false,"There is no RemoteDataType for body with id: " << id << "!")
           return NULL;
        }
    }

    LocalDataType * getLocalBodyData(RigidBodyType * body){
        return  getLocalBodyData(body->m_id);
    }

    LocalDataType * getLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localBodies.find(id);
        if(it != m_localBodies.end()){
           return (&it->second);
        }else{
           //ASSERTMSG(false,"There is no LocalDataType for body with id: " << id << "!")
           return NULL;
        }
    }

    bool eraseLocalBodyData(RigidBodyType * body){
        return eraseLocalBodyData(body->m_id);
    }
    bool eraseLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localBodies.find(id);
        if(it != m_localBodies.end()){
            m_localBodies.erase(it);
            return true;
        }
        return false;
    }

    bool eraseRemoteBodyData(RigidBodyType * body){
        return eraseRemoteBodyData(body->m_id);
    }

    bool eraseRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteBodies.find(id);
        if(it != m_remoteBodies.end()){
            m_remoteBodies.erase(it);
            return true;
        }
        return false;
    }

protected:
    RankIdType m_neighbourRank; // This is the rank to which this data structure belongs!

    /** Private because we do not want that the operator[] is called anywhere in these maps!
    * Not good behaviour if oeprator [] is used, as new element get inserted easily!
    */
    RemoteBodiesMapType m_remoteBodies;

    LocalBodiesMapType m_localBodies;
};


/**
* @brief This class is used in the NeighbourMap class as data structure for the general communication of bodies
*/
namespace NeighbourDataBodyCommunication_impl{
    // All overlapping remote bodies
    // These bodies need an update from the neighbour.
    // If no update move body to m_garbageBodyList list
    struct RemoteData{
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        RemoteData(RigidBodyType * body):m_pBody(body){};
        RigidBodyType * const m_pBody;
    };

    // All overlapping local bodies
    // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
    // NOTIFIED: send update to neighbour
    // MOVE: send whole body to neighbour (put into m_garbageBodyList)
    struct LocalData{
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        LocalData(RigidBodyType * body):m_pBody(body),m_commStatus(SEND_NOTIFICATION){};
        RigidBodyType * const m_pBody;
        enum {SEND_NOTIFICATION, SEND_UPDATE, SEND_REMOVE} m_commStatus;
    };
};

class NeighbourDataBodyCommunication: public NeighbourData< NeighbourDataBodyCommunication_impl::LocalData,
                                                            NeighbourDataBodyCommunication_impl::RemoteData>
{
public:
    DEFINE_MPI_INFORMATION_CONFIG_TYPES
private:
    typedef  NeighbourData< NeighbourDataBodyCommunication_impl::LocalData,
                            NeighbourDataBodyCommunication_impl::RemoteData> NeighbourDataDerived;

public:

    NeighbourDataBodyCommunication(const RankIdType & neighbourRank): NeighbourDataDerived(neighbourRank){};

    typedef NeighbourDataDerived::RemoteIterator RemoteIterator;
    typedef NeighbourDataDerived::LocalIterator LocalIterator;

    typedef NeighbourDataDerived::LocalDataType LocalDataType;
    typedef NeighbourDataDerived::RemoteDataType RemoteDataType;

    void clear(){
        ERRORMSG("We should not execute this!");
    }
};

#endif // NeighbourDataBodyCommunication_hpp

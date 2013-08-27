#ifndef NeighbourMap_hpp
#define NeighbourMap_hpp

#include <vector>
#include <list>
#include <type_traits>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"
#include "RigidBodyContainer.hpp"


template<typename TDynamicsSystem,typename TRankId>
class NeighbourData {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig   DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef TRankId RankIdType;

    struct RemoteData{
        RemoteData(RigidBodyType * body):m_body(body){};
        RigidBodyType * m_body;
    };
    struct LocalData{
        LocalData(RigidBodyType * body):m_body(body),m_commStatus(SEND_NOTIFICATION){};
        RigidBodyType * m_body;
        enum {SEND_NOTIFICATION, SEND_UPDATE, SEND_REMOVE} m_commStatus;
    };

    typedef std::map<typename RigidBodyType::RigidBodyIdType, RemoteData * > RemoteBodiesMapType;
    typedef typename RemoteBodiesMapType::iterator RemoteIterator;

    typedef std::map<typename RigidBodyType::RigidBodyIdType, LocalData * > LocalBodiesMapType;
    typedef typename LocalBodiesMapType::iterator LocalIterator;


    NeighbourData(RankIdType neighbourRank): m_neighbourRank(neighbourRank){};
    ~NeighbourData(){
        for(auto it =m_remoteBodies.begin(); it != m_remoteBodies.end(); it++ ){
            delete it->second;
        }
        m_remoteBodies.clear();

        for(auto it =m_localBodies.begin(); it != m_localBodies.end(); it++ ){
            delete it->second;
        }
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

    std::pair<LocalIterator, bool> addLocalBodyData(RigidBodyType * body){

        std::pair<LocalIterator, bool>  res = m_localBodies.insert(
                    typename LocalBodiesMapType::value_type(body->m_id, (LocalData*) NULL )
                    );

        if(res.second){ //if insert set the pointer
            res.first->second = new LocalData(body);
        }

        return res;
    }


    std::pair<RemoteIterator, bool> addRemoteBodyData(RigidBodyType * body){

        std::pair<RemoteIterator, bool> res = m_remoteBodies.insert(
                    typename RemoteBodiesMapType::value_type(body->m_id,(RemoteData*) NULL)
                    );

        if(res.second){ //if insert set the pointer
            res.first->second = new RemoteData(body);
        }

        return res;
    }

    RemoteData * getRemoteBodyData(RigidBodyType * body){
        return  getRemoteBodyData(body->m_id);
    }

    RemoteData * getRemoteBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_remoteBodies.find(id);
        if(it != m_remoteBodies.end()){
           return (it->second);
        }else{
           ASSERTMSG(false,"There is no RemoteData for body with id: " << id << "!")
           return NULL;
        }
    }

    LocalData * getLocalBodyData(RigidBodyType * body){
        return  getLocalBodyData(body->m_id);
    }

    LocalData * getLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localBodies.find(id);
        if(it != m_localBodies.end()){
           return (it->second);
        }else{
           ASSERTMSG(false,"There is no LocalData for body with id: " << id << "!")
           return NULL;
        }
    }

    bool deleteLocalBodyData(RigidBodyType * body){
        return deleteLocalBodyData(body->m_id);
    }
    bool deleteLocalBodyData(typename RigidBodyType::RigidBodyIdType id){
        auto it = m_localBodies.find(id);
        if(it != m_localBodies.end()){
            delete it->second;
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
            delete it->second;
            m_remoteBodies.erase(it);
            return true;
        }
        return false;
    }

    void cleanUp();

private:
    TRankId m_neighbourRank; // This is the rank to which this data structure belongs!

    // Private because we do not want that the operator[] is called anywhere in these maps!
    // Not good behaviour if oeprator [] is used, as new element get inserted easily!

    RemoteBodiesMapType m_remoteBodies; // All overlapping remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    LocalBodiesMapType m_localBodies; // All overlapping local bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)
};

template<typename TDynamicsSystem, typename TBodyToInfoMap>
class NeighbourMap {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    // Body info definitions
    typedef TBodyToInfoMap BodyInfoMapType;
    typedef typename BodyInfoMapType::DataType BodyInfoType;
    typedef typename BodyInfoType::RankIdType RankIdType;
    typedef typename BodyInfoType::RankToFlagsType RankToFlagsType;

    // Neighbour data definitions
    typedef NeighbourData<DynamicsSystemType,RankIdType> DataType;
    typedef std::map<RankIdType, DataType * > Type;
    typedef typename Type::iterator iterator;

    NeighbourMap(RankIdType rank, BodyInfoMapType & bodyToInfo): m_rank(rank), m_bodyToInfo(bodyToInfo){};

    ~NeighbourMap(){
        for(auto it = m_nbDataMap.begin(); it != m_nbDataMap.end(); it++){
            delete it->second;
        }
        m_nbDataMap.clear();
    }


    std::pair<iterator,bool> insert(const RankIdType & rank){
        std::pair<typename Type::iterator,bool> res = m_nbDataMap.insert( typename Type::value_type(rank, (DataType*)NULL));
        if(res.second){
            res.first->second = new DataType(rank);
        }
        return res;
    }

    template<typename TIterator>
    void addLocalBodyExclusive(RigidBodyType * body, TIterator neighbourRanksBegin, TIterator neighbourRanksEnd);

    void cleanUp();

    inline DataType * getNeighbourData(const RankIdType & rank){
        auto it = m_nbDataMap.find(rank);
        if(it != m_nbDataMap.end()){
           return (it->second);
        }else{
           ASSERTMSG(false,"There is no NeighbourData for rank: " << rank << "!")
           return NULL;
        }
    }

private:
    RankIdType m_rank;
    Type m_nbDataMap;

    BodyInfoMapType & m_bodyToInfo;

};

template<typename TDynamicsSystem, typename TBodyToInfoMap>
template< typename TIterator>
void NeighbourMap<TDynamicsSystem,TBodyToInfoMap>::addLocalBodyExclusive(RigidBodyType * body,
                                                                          TIterator neighbourRanksBegin,
                                                                          TIterator neighbourRanksEnd)
{
    // Add this local body exclusively to the given neighbours

    BodyInfoType * bodyInfo = m_bodyToInfo.getBodyInfo(body);

    // Loop over all incoming  ranks
    TIterator rankIt;
    for( rankIt = neighbourRanksBegin;rankIt!= neighbourRanksEnd;rankIt++){
        // insert the new element into body info --> (rank, flags)
        std::pair<typename RankToFlagsType::iterator,bool> res =
                   bodyInfo->m_neighbourRanks.insert(
                                        typename RankToFlagsType::value_type(*rankIt,typename BodyInfoType::Flags())
                                                       );

        res.first->second.m_bOverlaps = true; // set the Flags for the existing or the newly inserted entry (rank,flags)

        typename DataType::LocalIterator pairlocalDataIt;
        if(res.second){//if inserted we need to add this body to the underlying neighbour data
           //add to the data
           auto pairlocalDataIt = this->getNeighbourData(*rankIt)->addLocalBodyData(body);
           ASSERTMSG(pairlocalDataIt.second, "Insert to neighbour data rank: " << *rankIt << " in process rank: " <<m_rank << " failed!");
           pairlocalDataIt.first->second->m_commStatus = DataType::LocalData::SEND_NOTIFICATION; // No need because is set automatically in constructor
        }else{
            ASSERTMSG(this->getNeighbourData(*rankIt)->getLocalBodyData(body),"body with id "<<body->m_id << " in neighbour structure rank: " << *rankIt << " does not exist?" );
            ASSERTMSG(this->getNeighbourData(*rankIt)->getLocalBodyData(body)->m_commStatus == DataType::LocalData::SEND_UPDATE,
                      "m_commStatus for body with id: " << body->m_id << " in neighbour structure rank: " << *rankIt << "should be in update mode!");
        }


    }

    // Clean up of the neighbour structure is done after communication!
    // We cannot already remove the local bodies from the neighbours for
    // which the flag m_bOverlaps = false because, this needs to be communicated first! (such that any neighbour does not request any update anymore!)
    // So set the flags for this body to SEND_REMOVE for all his neighbours which have m_bOverlaps = false

    for( typename RankToFlagsType::iterator rankToFlagsIt = bodyInfo->m_neighbourRanks.begin();
        rankToFlagsIt != bodyInfo->m_neighbourRanks.end(); rankToFlagsIt++ ){

        if( rankToFlagsIt->second.m_bOverlaps == false){

            typename DataType::LocalData * localData =
            this->getNeighbourData(rankToFlagsIt->first)->getLocalBodyData(body);

            localData->m_commStatus = DataType::LocalData::SEND_REMOVE;

        }

    }

}

template<typename TDynamicsSystem, typename TBodyToInfoMap>
void NeighbourMap<TDynamicsSystem,TBodyToInfoMap>::cleanUp()
{
    for( typename BodyInfoType::iterator bodyInfoIt = m_bodyToInfo.begin(); bodyInfoIt != m_bodyToInfo.end(); bodyInfoIt++){

        // Loop over all (ranks,flags) for this body (remove the necessary ones)
        typename RankToFlagsType::iterator rankToFlagsIt = bodyInfoIt->second->m_NeighbourRanks.begin();
        while(  rankToFlagsIt != bodyInfoIt->second->m_NeighbourRanks.end() ){
            if( rankToFlagsIt->second->m_bOverlaps == false){

                // we need to remove this body from this rank
               this->getNeighbourData(rankToFlagsIt->first)->removeLocalBodyData(bodyInfoIt->first);

                //remove the rank from the rankToFlag list of THIS Body
                bodyInfoIt->second->m_NeighbourRanks.erase(rankToFlagsIt++);    // Note the post increment here.
                                                       // This increments 'rankToFlagsIt' and returns a copy of
                                                       // the original 'rankToFlagsIt' to be used by erase()
            }else{

                //set the flag to false, so that next time it will be removed if no other decision taken above
               rankToFlagsIt->second->m_bOverlaps == false;
               ++rankToFlagsIt;
            }
        }
    }
}


#endif

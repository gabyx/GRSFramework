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

    struct RemoteOverlapData{
        RemoteOverlapData():m_body(NULL){};
        RigidBodyType * m_body;
    };
    struct LocalOverlapData{
        LocalOverlapData():m_body(NULL),m_commStatus(SEND_NOTIFICATION){};
        RigidBodyType * m_body;
        enum {SEND_NOTIFICATION, SEND_UPDATE, SEND_REMOVE} m_commStatus;
    };
    typedef std::map<typename RigidBodyType::RigidBodyIdType, LocalOverlapData > LocalBodiesMapType;


    NeighbourData(RankIdType neighbourRank): m_neighbourRank(neighbourRank){};

    typename LocalBodiesMapType::iterator addLocalBody(RigidBodyType * body){

        std::pair<typename LocalBodiesMapType::iterator, bool> res = m_localBodies.insert(
                    typename LocalBodiesMapType::value_type(body->m_id,LocalOverlapData())
                    );

        if(res.second){ //if insert set the pointer
            res.first->second.m_body = body;
        }

        return res.first;
    }


    bool removeLocalBody(RigidBodyType * body){
        typename LocalBodiesMapType::size_type e = m_localBodies.erase(body->m_id);
        return e>0;
    }
    bool removeLocalBody(typename RigidBodyType::RigidBodyIdType id){
        typename LocalBodiesMapType::size_type e = m_localBodies.erase(id);
        return e>0;
    }

    bool removeRemoteBody(RigidBodyType * body){
        typename LocalBodiesMapType::size_type e = m_remoteBodies.erase(body->m_id);
        return e>0;
    }
    bool removeRemoteBody(typename RigidBodyType::RigidBodyIdType id){
        typename LocalBodiesMapType::size_type e = m_remoteBodies.erase(id);
        return e>0;
    }

    typedef std::map<typename RigidBodyType::RigidBodyIdType, LocalOverlapData > RemoteBodiesMapType;
    RemoteBodiesMapType m_remoteBodies; // All overlapping remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    LocalBodiesMapType m_localBodies; // All overlapping local bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)



    void cleanUp();

private:
    TRankId m_neighbourRank; // This is the rank to which this data structure belongs!
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
    ~NeighbourMap(){}


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
        typename Type::iterator it = m_nbDataMap.find(rank);
        ASSERTMSG(it != m_nbDataMap.end(),"There is no NeighbourData for rank: " << rank << "!")
        return (it->second);
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

        typename DataType::LocalBodiesMapType::iterator localOverlapIt;
        if(res.second){//if inserted we need to add this body to the underlying neighbour data
           //add to the data
           typename DataType::LocalBodiesMapType::iterator localOverlapIt =
           this->getNeighbourData(*rankIt)->addLocalBody(body);

           localOverlapIt->second.m_commStatus = DataType::LocalOverlapData::SEND_NOTIFICATION; // No need because is set automatically in constructor
        }


    }

    // Clean up of the neighbour structure is done after communication!
    // We cannot already remove the local bodies from the neighbours for
    // which the flag m_bOverlaps = false because, this needs to be communicated first! (such that any neighbour does not request any update anymore!)
    // So set the flags for this body to SEND_REMOVE for all his neighbours which have m_bOverlaps = false

    for( typename RankToFlagsType::iterator rankToFlagsIt = bodyInfo->m_neighbourRanks.begin();
        rankToFlagsIt != bodyInfo->m_neighbourRanks.end(); rankToFlagsIt++ ){

        if( rankToFlagsIt->second.m_bOverlaps == false){

            typename DataType::LocalBodiesMapType::iterator localOverlapIt =
            this->getNeighbourData(rankToFlagsIt->first)->m_localBodies.find(body->m_id);

            localOverlapIt->second.m_commStatus = DataType::LocalOverlapData::SEND_REMOVE;

        }

    }

}

template<typename TDynamicsSystem, typename TBodyToInfoMap>
void NeighbourMap<TDynamicsSystem,TBodyToInfoMap>::cleanUp()
{
    for( typename BodyInfoType::iterator bodyInfoIt = m_bodyToInfo.begin(); bodyInfoIt != m_bodyToInfo.end(); bodyInfoIt++){

        // Loop over all (ranks,flags) for this body (remove the necessary ones)
        typename RankToFlagsType::iterator rankToFlagsIt = bodyInfoIt->second.m_NeighbourRanks.begin();
        while(  rankToFlagsIt != bodyInfoIt->second.m_NeighbourRanks.end() ){
            if( rankToFlagsIt->second.m_bOverlaps == false){

                // we need to remove this body from this rank
               this->getNeighbourData(rankToFlagsIt->first)->removeLocalBody(bodyInfoIt->first);

                //remove the rank from the rankToFlag list of THIS Body
                bodyInfoIt->secondm_NeighbourRanks.erase(rankToFlagsIt++);    // Note the post increment here.
                                                       // This increments 'rankToFlagsIt' and returns a copy of
                                                       // the original 'rankToFlagsIt' to be used by erase()
            }else{

                //set the flag to false, so that next time it will be removed if no other decision taken above
               rankToFlagsIt->second.m_bOverlaps == false;
               ++rankToFlagsIt;
            }
        }
    }
}


#endif

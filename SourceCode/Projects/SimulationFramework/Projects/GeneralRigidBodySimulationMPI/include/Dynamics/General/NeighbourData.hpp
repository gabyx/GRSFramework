#ifndef NeighbourData_hpp
#define NeighbourData_hpp

#include <vector>
#include <list>

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

    NeighbourData(RankIdType neighbourRank): m_neighbourRank(neighbourRank){};

    struct RemoteOverlapData{
        RemoteOverlapData():m_body(NULL){};
        RigidBodyType * m_body;
    };

    struct LocalOverlapData{
        LocalOverlapData():m_body(NULL),m_commStatus(NOT_NOTFIED){};
        RigidBodyType * m_body;
        enum {NOT_NOTFIED, NOTIFIED, MOVE} m_commStatus;
    };

    void addLocalBody(RigidBodyType * body, RankIdType owningRank){

        std::pair<typename LocalBodiesMapType::iterator, bool> res = m_localBodies.insert(
                    typename LocalBodiesMapType::value_type(body->m_id,LocalOverlapData())
                    );

        if(res.second){ //if insert set the pointer
            res.first->second.m_body = body;
        }

        // If this is the owning process then set the comm status to MOVE,
        // such that this gets notified in the next communication !
        if(owningRank == m_neighbourRank){
            res.first->second.m_commStatus = LocalOverlapData::MOVE;
        }
    }

    void removeLocalBody(RigidBodyType * body){
        typename LocalBodiesMapType::size_type e = m_localBodies.erase(body->m_id);
        ASSERTMSG(e!=0,"Tried to delete body id: " << body->m_id << " which failed ")
    }

    typedef std::map<typename RigidBodyType::RigidBodyIdType, LocalOverlapData > RemoteBodiesMapType;
    RemoteBodiesMapType m_remoteBodies; // All overlapping remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    typedef std::map<typename RigidBodyType::RigidBodyIdType, LocalOverlapData > LocalBodiesMapType;
    LocalBodiesMapType m_localBodies; // All overlapping local bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)



    void cleanUp();

private:
    TRankId m_neighbourRank; // This is the rank to which this data structure belongs!
};

template<typename TDynamicsSystem, typename TRankId>
class NeighbourMap {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef TRankId RankIdType;
    typedef NeighbourData<DynamicsSystemType,RankIdType> NeighbourDataType;
    typedef std::map<RankIdType, NeighbourDataType * > NeighbourDataMapType;


    NeighbourMap(RankIdType rank): m_rank(rank){};


    template< template<typename> class TList>
    void addLocalBodyExclusive( RigidBodyType * body, const TList<RankIdType> & neighbourRanks, RankIdType owningRank );

    //void addLocalBodyExclusive( RigidBodyType * body, const std::vector<RankIdType> & neighbourRanks, RankIdType owningRank );


    void addNewNeighbourData(const RankIdType & rank){
        std::pair<typename NeighbourDataMapType::iterator,bool> res =
            m_nbDataMap.insert(
                    typename NeighbourDataMapType::value_type(rank, new NeighbourDataType(rank))
                                );
        ASSERTMSG(res.second == true,"You inserted a NeighbourData which is already existing for this rank: "<<rank);
    }


    inline NeighbourDataType & operator[](const RankIdType & rank){
        typename NeighbourDataMapType::iterator it = m_nbDataMap.find(rank);
        ASSERTMSG(it != m_nbDataMap.end(),"There is no NeighbourData for rank: " << rank << "!")
        return *(it->second);
    }

private:
    RankIdType m_rank;
    NeighbourDataMapType m_nbDataMap;

    /**
    * Data structure in the Map: Rank -> Flags, Flags define the behaviour what needs to be done with this Body.
    * m_bToRemove: Used to decide if body is removed from the corresponding neigbourS
    */
    struct Flags{
        Flags():m_bToRemove(true){};
        bool m_bToRemove;
    };
    typedef  std::map<RankIdType, Flags> RankToFlagsType;
    typedef  std::map<typename RigidBodyType::RigidBodyIdType, RankToFlagsType > BodyToNeighbourProcessType;
    BodyToNeighbourProcessType m_bodyToNeighbourProcess; ///< map which gives all overlapping processes

};

template<typename TDynamicsSystem, typename TRankId>
template< template<typename> class TList>
void NeighbourMap<TDynamicsSystem,TRankId>::addLocalBodyExclusive(RigidBodyType * body,
                                                                   const TList<RankIdType> & neighbourRanks,
                                                                   RankIdType owningRank )
{
    // Add this local body exclusively to the given neighbours

    RankToFlagsType & rankToFlags = m_bodyToNeighbourProcess[body->m_id];
    // if it does not exist a map RankToFlagsType will be constructed.

    // itAN: assigned neighbours (map wih ranks)

    // Loop over all incoming  ranks
    typename TList<RankIdType>::const_iterator rankIt;
    for( rankIt = neighbourRanks.begin();rankIt!= neighbourRanks.end();rankIt++){
        // insert the new element (rank, flags)
        std::pair<typename RankToFlagsType::iterator,bool> res = rankToFlags.insert( typename RankToFlagsType::value_type(*rankIt,Flags())  );
        res.first->second.m_bToRemove = false; // set the Flags for the existing or the newly inserted entry (rank,flags)
        if(res.second){//if inserted we need to add this body to the underlying neighbour data
           //add to the data

           this->operator[](*rankIt).addLocalBody(body,owningRank);
        }

    }

    // Loop over all (ranks,flags) for this body (remove the necessary ones)
    typename RankToFlagsType::iterator rankToFlagsIt = rankToFlags.begin();
    while(  rankToFlagsIt != rankToFlags.end() ){
        if( rankToFlagsIt->second.m_bToRemove == true){

            // we need to remove this body from this rank
           this->operator[](rankToFlagsIt->first).removeLocalBody(body);

            //remove the rank from the rankToFlag list of THIS Body
            rankToFlags.erase(rankToFlagsIt++);    // Note the post increment here.
                                                   // This increments 'rankToFlagsIt' and returns a copy of
                                                   // the original 'rankToFlagsIt' to be used by erase()
        }else{

            //set the flag to remove to true, so that next time it will be removed if no other decision taken above
           rankToFlagsIt->second.m_bToRemove == true;
           ++rankToFlagsIt;
        }
    }



}



#endif

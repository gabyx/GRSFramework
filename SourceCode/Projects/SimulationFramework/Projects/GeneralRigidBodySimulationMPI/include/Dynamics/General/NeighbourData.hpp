#ifndef NeighbourData_hpp
#define NeighbourData_hpp

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

template<typename TDynamicsSystem, typename TBodyToInfoMap>
class NeighbourMap {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)


    typedef TBodyToInfoMap BodyToInfoMapType;
    typedef typename std::remove_pointer<typename BodyToInfoMapType::mapped_type>::type BodyInfoType;
    typedef typename BodyInfoType::RankIdType RankIdType;

    typedef NeighbourData<DynamicsSystemType,RankIdType> NeighbourDataType;
    typedef std::map<RankIdType, NeighbourDataType * > NeighbourDataMapType;


    NeighbourMap(RankIdType rank, BodyToInfoMapType & bodyToInfo): m_rank(rank), m_bodyToInfo(bodyToInfo){};
    ~NeighbourMap(){
        //Delete all neighbour datas
        typename NeighbourDataMapType::iterator it;
        for(it=m_nbDataMap.begin();it != m_nbDataMap.end();it++){
            delete it->second;
        }
    }


    void addNewNeighbourData(const RankIdType & rank){
        std::pair<typename NeighbourDataMapType::iterator,bool> res =
            m_nbDataMap.insert(
                    typename NeighbourDataMapType::value_type(rank, new NeighbourDataType(rank))
                                );
        ASSERTMSG(res.second == true,"You inserted a NeighbourData which is already existing for this rank: "<<rank);
    }

    template< template<typename,typename> class TList, typename Alloc>
    void addLocalBodyExclusive( RigidBodyType * body, const TList<RankIdType, Alloc> & neighbourRanks);

    inline NeighbourDataType & getNeighbourData(const RankIdType & rank){
        typename NeighbourDataMapType::iterator it = m_nbDataMap.find(rank);
        ASSERTMSG(it != m_nbDataMap.end(),"There is no NeighbourData for rank: " << rank << "!")
        return *(it->second);
    }


    inline NeighbourDataType & operator[](const RankIdType & rank){
        typename NeighbourDataMapType::iterator it = m_nbDataMap.find(rank);
        ASSERTMSG(it != m_nbDataMap.end(),"There is no NeighbourData for rank: " << rank << "!")
        return *(it->second);
    }

private:
    RankIdType m_rank;
    NeighbourDataMapType m_nbDataMap;

    BodyToInfoMapType & m_bodyToInfo;

};

template<typename TDynamicsSystem, typename TBodyToInfoMap>
template< template<typename,typename> class TList, typename Alloc>
void NeighbourMap<TDynamicsSystem,TBodyToInfoMap>::addLocalBodyExclusive(RigidBodyType * body,
                                                                   const TList<RankIdType,Alloc> & neighbourRanks)
{
    // Add this local body exclusively to the given neighbours

//    RankToFlagsType & rankToFlags = m_bodyToNeighbourProcess[body->m_id];
//    // if it does not exist a map RankToFlagsType will be constructed.
//
//    // itAN: assigned neighbours (map wih ranks)
//
//    // Loop over all incoming  ranks
//    typename TList<RankIdType,Alloc>::const_iterator rankIt;
//    for( rankIt = neighbourRanks.begin();rankIt!= neighbourRanks.end();rankIt++){
//        // insert the new element (rank, flags)
//        std::pair<typename RankToFlagsType::iterator,bool> res = rankToFlags.insert( typename RankToFlagsType::value_type(*rankIt,Flags())  );
//        res.first->second.m_bToRemove = false; // set the Flags for the existing or the newly inserted entry (rank,flags)
//        if(res.second){//if inserted we need to add this body to the underlying neighbour data
//           //add to the data
//
//           this->getNeighbourData(*rankIt).addLocalBody(body);
//        }
//
//    }
//
//    // Loop over all (ranks,flags) for this body (remove the necessary ones)
//    typename RankToFlagsType::iterator rankToFlagsIt = rankToFlags.begin();
//    while(  rankToFlagsIt != rankToFlags.end() ){
//        if( rankToFlagsIt->second.m_bToRemove == true){
//
//            // we need to remove this body from this rank
//           this->getNeighbourData(rankToFlagsIt->first).removeLocalBody(body);
//
//            //remove the rank from the rankToFlag list of THIS Body
//            rankToFlags.erase(rankToFlagsIt++);    // Note the post increment here.
//                                                   // This increments 'rankToFlagsIt' and returns a copy of
//                                                   // the original 'rankToFlagsIt' to be used by erase()
//        }else{
//
//            //set the flag to remove to true, so that next time it will be removed if no other decision taken above
//           rankToFlagsIt->second.m_bToRemove == true;
//           ++rankToFlagsIt;
//        }
//    }
}



#endif

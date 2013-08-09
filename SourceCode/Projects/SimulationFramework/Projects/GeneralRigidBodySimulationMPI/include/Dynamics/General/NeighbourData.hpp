#ifndef NeighbourData_hpp
#define NeighbourData_hpp

#include <vector>
#include <list>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"
#include "RigidBodyContainer.hpp"


template<typename TDynamicsSystem>
class NeighbourData {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig   DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    struct RemoteOverlapData{
        RigidBodyType * m_body;
    };

    struct LocalOverlapData{
        RigidBodyType * m_body;
        enum {NOT_NOTFIED, NOTIFIED, MOVE};
    };

    std::map<typename RigidBodyType::RigidBodyIdType, RemoteOverlapData > m_remoteBodies; // All overlapping remote bodies
        // These bodies need an update from the neighbour.
        // If no update move body to m_garbageBodyList list

    std::map<typename RigidBodyType::RigidBodyIdType, LocalOverlapData > m_localBodies; // All overlapping local bodies
        // NOT_NOTIFIED: send whole body to the neighbour (change to NOTIFIED)
        // NOTIFIED: send update to neighbour
        // MOVE: send whole body to neighbour (put into m_garbageBodyList)

    void cleanUp();

private:

};

template<typename TDynamicsSystem, typename TRankId>
class NeighbourMap {
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef TRankId RankIdType;
    typedef std::map<RankIdType, NeighbourData<DynamicsSystemType> > NeighbourDataMapType;




    NeighbourMap(){
    }

    void addLocalBodyExclusive(std::vector<RankIdType> neighbours);

    void addNewNeighbourData(const RankIdType & rank){
        std::pair<typename NeighbourDataMapType::iterator,bool> res =
            m_nbDataMap.insert(
                               std::pair<RankIdType,NeighbourData<DynamicsSystemType> >(rank,NeighbourData<DynamicsSystemType>() )
                                );
        ASSERTMSG(res.second == true,"You inserted an NeighbourData which is already existing for this rank: "<<rank);
    }

    inline NeighbourData<DynamicsSystemType> & operator[](const RankIdType & rank){
        return m_nbDataMap[rank];
    }

private:

    NeighbourDataMapType m_nbDataMap;
    std::map<typename RigidBodyType::RigidBodyIdType, std::map<RankIdType, int> > m_bodyToOverlapProcess; ///< map which gives all overlapping processes

};




#endif

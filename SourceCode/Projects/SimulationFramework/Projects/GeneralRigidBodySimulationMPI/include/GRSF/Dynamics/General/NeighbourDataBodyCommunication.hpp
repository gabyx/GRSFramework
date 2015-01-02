#ifndef GRSF_Dynamics_General_NeighbourDataBodyCommunication_hpp
#define GRSF_Dynamics_General_NeighbourDataBodyCommunication_hpp

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/General/NeighbourData.hpp"

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

    using RemoteIterator = NeighbourDataDerived::RemoteIterator;
    using LocalIterator = NeighbourDataDerived::LocalIterator;

    using LocalDataType = NeighbourDataDerived::LocalDataType;
    using RemoteDataType = NeighbourDataDerived::RemoteDataType;

    void clear(){
        ERRORMSG("We should not execute this!");
    }
};

#endif // NeighbourDataBodyCommunication_hpp

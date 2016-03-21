#ifndef GRSF_Dynamics_Inclusion_NeighbourDataInclusionCommunication_hpp
#define GRSF_Dynamics_Inclusion_NeighbourDataInclusionCommunication_hpp


#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Dynamics/General/NeighbourData.hpp"

class ContactGraphNodeDataSplitBody;

namespace NeighbourDataInclusionCommunication_impl{

    /**
    * Remote Data: Stores the remote splitted body. This data structure is useful to determine which remote body velocity updates need to be
    * sent to the neighbour rank for the splitbody update.
    * These struct is used for Remote-Local Contacts, and is built before sending each neighbour all ids of these remoteDatas
    */
    struct RemoteData{
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        RemoteData(RigidBodyType * body):
            m_pBody(body){
        };

        RigidBodyType * const m_pBody; ///< remote body
    };

    /**
    * Local Data: Stores the local split body and its node, this data structure is usefull to determine
    * from which neighbour we get updated velocities for the node.
    * These structs are built during receiving of the messages (see above)
    */
    struct LocalData{
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        LocalData(RigidBodyType * body):m_pBody(body),m_pSplitBodyNode(nullptr){};
        RigidBodyType * const m_pBody; ///< local body (e.g id=10, neighbour has RemoteData with id=10 as well!)
        ContactGraphNodeDataSplitBody * m_pSplitBodyNode;
    };


    /**
    * Remote Data: Stores the node which belongs to a remote-remote contact and a bool which defines how this remote-remote contact is notified
    * to the neighbour rank.
    */
    template<typename TNodeData>
    struct RemoteDataTemp{
        RemoteDataTemp(bool firstBodyIsSplitted, bool sendSplit,  TNodeData * pNodeData):
            m_firstBodyIsSplitted(firstBodyIsSplitted), m_sendSplit(sendSplit) , m_pNodeData(pNodeData){};

        bool m_firstBodyIsSplitted;  //Body 1 or 2 which is splitted
        bool m_sendSplit; // Decides if split message or remote node message is sent!


        TNodeData * m_pNodeData;

    };


};

/**
* @brief This class is used in the NeighbourMap class as data structure for the communication in the inclusion solver
*/

//template<typename TNode>
class NeighbourDataInclusionCommunication:  public NeighbourData< NeighbourDataInclusionCommunication_impl::LocalData,
                                                                  NeighbourDataInclusionCommunication_impl::RemoteData>
{
public:
    DEFINE_MPI_INFORMATION_CONFIG_TYPES


private:
    typedef NeighbourData< NeighbourDataInclusionCommunication_impl::LocalData,
                           NeighbourDataInclusionCommunication_impl::RemoteData> NeighbourDataDerived;

public:

    NeighbourDataInclusionCommunication(const RankIdType &neighbourRank): NeighbourDataDerived(neighbourRank){};

    using RemoteIterator = NeighbourDataDerived::RemoteIterator;
    using LocalIterator = NeighbourDataDerived::LocalIterator;

    using LocalDataType = NeighbourDataDerived::LocalDataType;
    using RemoteDataType = NeighbourDataDerived::RemoteDataType;

};


#endif // NeighbourDataInclusionCommunication_hpp


#ifndef GRSF_dynamics_inclusion_InclusionCommunicator_hpp
#define GRSF_dynamics_inclusion_InclusionCommunicator_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/LogDefines.hpp"


#include "GRSF/dynamics/general/MPICommunication.hpp"
#include "GRSF/dynamics/general/BodyCommunicator.hpp"

#include "GRSF/dynamics/general/NeighbourMap.hpp"
#include "GRSF/dynamics/inclusion/NeighbourDataInclusionCommunication.hpp"

#include "GRSF/dynamics/general/MPIMessages.hpp"

#include InclusionSolverSettings_INCLUDE_FILE

template<typename TCombo>
class InclusionCommunicator {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    //Cannot template on this type, template cyclic dependency because ContactGraph also template on InlcusionComm
    // Dont template on InclusionCommunicator and ContactGraph
    // Cannnot typedef more dependent types in ContactGraph, because doing so,
    using ContactGraphType = typename TCombo::ContactGraphType;

    using ProcessCommunicatorType = MPILayer::ProcessCommunicator                                     ;
    using ProcessInfoType = typename ProcessCommunicatorType::ProcessInfoType                          ;
    using ProcessTopologyType = typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType     ;

    using RigidBodyContainerType = typename DynamicsSystemType::RigidBodySimContainerType                     ;
    using GlobalGeometryMapType = typename DynamicsSystemType::GlobalGeometryMapType                         ;

    //using NodeDataType = typename ContactGraphType::NodeDataType;
    using NeighbourMapType = NeighbourMap<NeighbourDataInclusionCommunication/*<NodeDataType> */>;

    InclusionCommunicator(std::shared_ptr< BodyCommunicator> pBodyComm,
                          std::shared_ptr< DynamicsSystemType> pDynSys ,
                          std::shared_ptr< ProcessCommunicatorType > pProcComm);


    void setContactGraph(ContactGraphType * pGraph){
        m_pContactGraph = pGraph;
    }


    ~InclusionCommunicator(){}

    void reset();
    void resetTopology();

    void communicateRemoteContacts(PREC currentSimulationTime);
    void communicateSplitBodyUpdate(unsigned int globalIterationNumber);
    void communicateSplitBodySolution(unsigned int globalIterationNumber);

    void buildCommunicatorGroups();

    bool communicateConvergence(bool converged);

    void resetAllWeightings();

    void resetForNextTimestep(){
        m_nbDataMap.emptyAllNeighbourData();
    }

    void setSettings(const InclusionSolverSettingsType & settings){
        m_settings = settings;
    }

    NeighbourMapType * getNeighbourMap(){return &m_nbDataMap;}

private:

    InclusionSolverSettingsType m_settings;


    /** Functions are executed in this order in communicateRemoteContacts() */
    void sendContactMessageToNeighbours();
    void receiveContactMessagesFromNeighbours();

    void sendBodyMultiplicityMessageToNeighbours();
    void recvBodyMultiplicityMessageFromNeighbours();



    /** Functions are executed in this order in communicateSplitBodyUpdate() */
    void sendUpdateSplitBodiesToNeighbours();
    void recvUpdateSplitBodiesFromNeighbours();

    void sendSolutionSplitBodiesToNeighbours();
    void recvSolutionSplitBodiesFromNeighbours();

    PREC m_currentSimulationTime;
    unsigned int m_globalIterationNumber;

    /**
    * The NeighbourMessageWrapperInclusion class needs access, to be able to serialize all together!
    */
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusionContact;
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusionMultiplicity;
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusionSplitBodyUpdate;
    template<typename TNeighbourCommunicator> friend class MPILayer::NeighbourMessageWrapperInclusionSplitBodySolution;


    MPILayer::NeighbourMessageWrapperInclusionContact< InclusionCommunicator > m_messageContact;
    MPILayer::NeighbourMessageWrapperInclusionMultiplicity< InclusionCommunicator > m_messageMultiplicity;
    MPILayer::NeighbourMessageWrapperInclusionSplitBodyUpdate< InclusionCommunicator > m_messageSplitBodyUpdate;
    MPILayer::NeighbourMessageWrapperInclusionSplitBodySolution< InclusionCommunicator > m_messageSplitBodySolution;

    std::shared_ptr< DynamicsSystemType >      m_pDynSys;
    std::shared_ptr< ProcessCommunicatorType > m_pProcComm;
    ContactGraphType *                         m_pContactGraph;
    std::shared_ptr< BodyCommunicator>         m_pBodyComm;

    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    typename ProcessTopologyType::NeighbourRanksListType m_nbRanks;

    std::set<RankIdType> m_nbRanksSendRecvRemote;
    /** If rank is in this set then there are remote bodies: send Velocities to rank,
                                                             recv BillateralUpdate from rank*/

    std::set<RankIdType> m_nbRanksSendRecvLocal;
    /** If rank is in this set then there are local bodies: recv Velocities from rank,
                                                            send BillateralUpdate to rank */

    RigidBodyContainerType & m_globalLocal;
    RigidBodyContainerType & m_globalRemote;



    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures

    Logging::Log *  m_pSimulationLog;

};


#include "GRSF/dynamics/inclusion/InclusionCommunicator.icc"


#endif

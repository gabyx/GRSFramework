#ifndef InclusionCommunicator_hpp
#define InclusionCommunicator_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"


#include "MPICommunication.hpp"
#include "BodyCommunicator.hpp"

#include "NeighbourMap.hpp"
#include "NeighbourDataInclusionCommunication.hpp"

#include "MPIMessages.hpp"

#include "InclusionSolverSettings.hpp"

template<typename Combo>
class InclusionCommunicator {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    //Cannot template on this type, template cyclic dependency because ContactGraph also template on InlcusionComm
    // Dont template on InclusionCommunicator and ContactGraph
    // Cannnot typedef more dependent types in ContactGraph, because doing so,
    typedef typename Combo::ContactGraphType ContactGraphType;

    typedef typename MPILayer::ProcessCommunicator                                      ProcessCommunicatorType;
    typedef typename ProcessCommunicatorType::ProcessInfoType                           ProcessInfoType;
    typedef typename ProcessCommunicatorType::ProcessInfoType::ProcessTopologyType      ProcessTopologyType;

    typedef typename DynamicsSystemType::RigidBodySimContainerType                      RigidBodyContainerType;
    typedef typename DynamicsSystemType::GlobalGeometryMapType                          GlobalGeometryMapType;


    typedef NeighbourMap<NeighbourDataInclusionCommunication> NeighbourMapType;

    InclusionCommunicator(boost::shared_ptr< BodyCommunicator> pBodyComm,
                          boost::shared_ptr< DynamicsSystemType> pDynSys ,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom);


    void setContactGraph(boost::shared_ptr< ContactGraphType> pGraph){
        m_pContactGraph = pGraph;
    }


    ~InclusionCommunicator(){}



    void communicateRemoteContacts(PREC currentSimulationTime);
    void communicateSplitBodyUpdate(unsigned int globalIterationNumber);
    void communicateSplitBodySolution(unsigned int globalIterationNumber);

    void resetAllWeightings();
    void initRemoteBodyVelocities();

    void clearNeighbourMap(){
        m_nbDataMap.emptyAllNeighbourData();
    }

    void setSettings(const InclusionSolverSettings & settings){
        m_Settings = settings;
    }

    NeighbourMapType * getNeighbourMap(){return &m_nbDataMap;}

private:

    InclusionSolverSettings m_Settings;


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

    boost::shared_ptr< DynamicsSystemType >      m_pDynSys;
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;
    boost::shared_ptr< ProcessInfoType >         m_pProcInfo;
    boost::shared_ptr< ContactGraphType >        m_pContactGraph;
    boost::shared_ptr< BodyCommunicator>         m_pBodyComm;

    RankIdType m_rank;

    ProcessTopologyType * m_pProcTopo;
    const typename ProcessTopologyType::NeighbourRanksListType & m_nbRanks;

    std::set<RankIdType> m_nbRanksSendRecvRemote;
    /** If rank is in this set then there are remote bodies: send Velocities to rank,
                                                             recv BillateralUpdate from rank*/

    std::set<RankIdType> m_nbRanksSendRecvLocal;
    /** If rank is in this set then there are local bodies: recv Velocities from rank,
                                                            send BillateralUpdate to rank */

    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;


    NeighbourMapType m_nbDataMap;   ///< map which gives all neighbour data structures

    Logging::Log *  m_pSimulationLog;

};


#include "InclusionCommunicator.icc"


#endif

#ifndef NeighbourCommunicator_hpp
#define NeighbourCommunicator_hpp


#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include <boost/shared_ptr.hpp>

#include "RigidBody.hpp"
#include "RigidBodyGarbageCollector.hpp"

#include "NeighbourData.hpp"
#include "MPICommunication.hpp"

template<typename TDynamicsSystem>
class NeighbourCommunicator{
public:

    typedef typename TDynamicsSystem::DynamicsSystemConfig              DynamicsSystemConfig;
    typedef TDynamicsSystem DynamicsSystemType;
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemConfig)

    typedef typename MPILayer::ProcessCommunicator<LayoutConfigType>    ProcessCommunicatorType;
    typedef typename DynamicsSystemType::RigidBodySimContainer          RigidBodyContainerType;

    NeighbourCommunicator(typename DynamicsSystemType::RigidBodySimContainer & globalLocal,
                          typename DynamicsSystemType::RigidBodySimContainer & globalRemote,
                          boost::shared_ptr< ProcessCommunicatorType > pProcCom)
                          : m_globalLocal(globalLocal), m_globalRemote(globalRemote), m_pProcCom(pProcCom)

    {
        if(Logging::LogManager::getSingletonPtr()->existsLog("SimulationLog")){
            m_pSimulationLog = Logging::LogManager::getSingletonPtr()->getLog("SimulationLog");
        }else{
            ERRORMSG("SimulationLog does not yet exist? Did you create it?")
        }


        // Initialize all NeighbourDatas

        const std::vector<unsigned int> & nbRanks = m_pProcCom->getProcessInfo()->getProcTopo()->getNeigbourRanks();
        for(int i=0;i< nbRanks.size();i++){
            LOG(m_pSimulationLog,"---> Add neighbour data for process rank: "<<nbRanks[i]<<std::endl;);
            m_nbDataMap[nbRanks[i]] = NeighbourData<DynamicsSystemConfig>();
        }
        m_pSimulationLog->logMessage("---> Initialized all NeighbourDatas");



    }


private:
    boost::shared_ptr< ProcessCommunicatorType > m_pProcCom;

    RigidBodyContainerType & m_globalRemote;
    RigidBodyContainerType & m_globalLocal;
    std::map<unsigned int, NeighbourData<DynamicsSystemConfig> > m_nbDataMap;

    RigidBodyGarbageCollector<DynamicsSystemConfig>  m_rigidBodyGC;

    Logging::Log *  m_pSimulationLog;

};

#endif

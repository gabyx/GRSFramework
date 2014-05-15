#ifndef MPITopologyBuilder_hpp
#define MPITopologyBuilder_hpp

#include <mpi.h>

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"


#include "MPIMessages.hpp"

#include "MPICommunication.hpp"

#include DynamicsSystem_INCLUDE_FILE

namespace MPILayer {

class TopologyBuilder{

    public:

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES

        typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;

        TopologyBuilder(boost::shared_ptr<DynamicsSystemType> pDynSys,
                        boost::shared_ptr<ProcessCommunicatorType > pProcCommunicator):
                            m_pDynSys(pDynSys), m_pProcCommunicator(pProcCommunicator) {

        }
    protected:
       boost::shared_ptr<DynamicsSystemType>  m_pDynSys;
       boost::shared_ptr<ProcessCommunicatorType> m_pProcCommunicator;
};

class GridTopologyBuilder : public TopologyBuilder {

      template<typename TTopologyBuilder >
      friend class TopologyBuilderMessageWrapperBodies;

     public:
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES
        typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;

        GridTopologyBuilder(boost::shared_ptr<DynamicsSystemType> pDynSys,
                            boost::shared_ptr<ProcessCommunicatorType > pProcCommunicator):
                            TopologyBuilder(pDynSys, pProcCommunicator)
        {}

        void rebuildTopology(){

            // Gather all body center of gravities and common AABB to master rank
            buildCurrentAABB();

            TopologyBuilderMessageWrapperBodies<GridTopologyBuilder> m(this);
            RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
            if(this->m_pProcCommunicator->hasMasterRank()){
                // Fill linear array with all ranks except master
                std::vector<RankIdType> ranks;
                unsigned int count=0;
                std::generate_n(std::back_inserter(ranks),
                                this->m_pProcCommunicator->getNProcesses(),
                                [&](){ if(count == masterRank){return (++count)++;}else{return count++;} });

                this->m_pProcCommunicator->recvMessageFromRanks(m, ranks, m.m_tag);

            }else{
                this->m_pProcCommunicator->sendMessageToNeighbourRank(m, masterRank, m.m_tag);
                this->m_pProcCommunicator->waitForAllNeighbourSends();
            }

            // Build new and optimal grid and broadcast
            if(this->m_pProcCommunicator->hasMasterRank()){
                buildGrid();
                // Broadcast grid
                // Install grid
            }else{
                // Receive Broadcast and install grid
            }


            // Order all bodies according to their rank in the new grid

            // Communicate all to all, bodies which are in other ranks then ours

            // Finished
        }

        void buildCurrentAABB(){
            m_currAABB = AABB();
            for(auto bodyIt = this->m_pDynSys->m_SimBodies.begin(); bodyIt != this->m_pDynSys->m_SimBodies.end(); bodyIt++){
                m_currAABB.unite((*bodyIt)->m_r_S);
            }
        }

        //only master rank
        void buildGrid(){



        }


     private:
        AABB m_currAABB;

        std::vector<AABB> m_rankAABBs;
        std::vector<Vector3> m_points;
};

}; //MPILayer

#endif


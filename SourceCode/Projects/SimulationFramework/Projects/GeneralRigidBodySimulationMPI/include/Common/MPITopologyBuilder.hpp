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

            // Broadcast all body center of gravities and common AABB to master rank
            buildCurrentAABB();

            TopologyBuilderMessageWrapperBodies<GridTopologyBuilder> m(this);
            if(this->m_pProcCommunicator->hasMasterRank()){
                this->m_pProcCommunicator->sendBroadcast(m);
            }else{
                this->m_pProcCommunicator->receiveBroadcast(m, m_pProcCommunicator->getMasterRank() );
            }

            // Build new and optimal grid and broadcast
            if(this->m_pProcCommunicator->hasMasterRank()){
                // Build grid
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

     private:
        AABB m_currAABB;

};

}; //MPILayer

#endif


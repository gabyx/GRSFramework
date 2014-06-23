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

        TopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                        std::shared_ptr<ProcessCommunicatorType > pProcCommunicator,
                        unsigned int nGlobalSimBodies):
                            m_pDynSys(pDynSys), m_pProcCommunicator(pProcCommunicator), m_nGlobalSimBodies(nGlobalSimBodies) {

        }
    protected:
       std::shared_ptr<DynamicsSystemType>  m_pDynSys;
       std::shared_ptr<ProcessCommunicatorType> m_pProcCommunicator;
};

class GridTopologyBuilder : public TopologyBuilder {

      template<typename TTopologyBuilder >
      friend class TopologyBuilderMessageWrapperBodies;

     public:
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
        DEFINE_MPI_INFORMATION_CONFIG_TYPES
        typedef typename MPILayer::ProcessCommunicator ProcessCommunicatorType;

        GridTopologyBuilder(std::shared_ptr<DynamicsSystemType> pDynSys,
                            std::shared_ptr<ProcessCommunicatorType > pProcCommunicator, unsigned int nGlobalSimBodies):
                            TopologyBuilder(pDynSys, pProcCommunicator,nGlobalSimBodies)
        {}

        void rebuildTopology(){

            // Gather all body center of gravities and common AABB to master rank
            // Compute Inertia tensor where all masses of the points are equal to 1 (gives the tensor for the geometric center)
            buildLocalStuff();




            TopologyBuilderMessageWrapperBodies<GridTopologyBuilder> m(this);
            RankIdType masterRank = this->m_pProcCommunicator->getMasterRank();
            if(this->m_pProcCommunicator->hasMasterRank()){

                // Rest global stuff
                m_aabb_glo.reset(); // Reset total AABB
                m_theta_G_glo.setZero();
                m_massPoints_glo.clear();
                m_initStates.clear();
                m_bodiesPerRank.clear();

                // Fill linear array with all ranks except master
                std::vector<RankIdType> ranks;
                unsigned int count=0;
                std::generate_n(std::back_inserter(ranks),
                                this->m_pProcCommunicator->getNProcesses(),
                                [&](){ if(count == masterRank){return (++count)++;}else{return count++;} });

                this->m_pProcCommunicator->receiveMessageFromRanks(m, ranks, m.m_tag);

                //Check if number of masspoints received is equal to the total in the simulation
                if(m_nGlobalSimBodies != m_massPoints_glo.size()){
                    ERRORMSG("m_nGlobalSimBodies: " << m_nGlobalSimBodies << "not equal to" << m_massPoints_glo.size() <<std::endl);
                }

                // Weight the geometric center
                m_r_G_glo /= m_massPoints_glo.size();

                buildGrid();

                // BroadCast information (Grid and InitalStates)



            }else{
                this->m_pProcCommunicator->sendMessageToRank(m, masterRank, m.m_tag);
            }

        }


        void buildLocalStuff(){

            m_aabb_loc.reset();
            m_Theta_I_loc.setZero();
            m_r_G_loc.setZero();

            // we copy all body points into the set m_points_loc
            // and add the number points according to the timesteps the predictor integrator is generating
            unsigned int nPointsPredictor = 4;


            // calculate geometric center and AABB and also inertia tensor (first with respect to interia frame,
            // then we move it with steiner formula to geometric center)
            for(auto bodyIt = this->m_pDynSys->m_SimBodies.begin(); bodyIt != this->m_pDynSys->m_SimBodies.end(); bodyIt++){

                m_aabb_loc.unite((*bodyIt)->m_r_S);

                m_r_G_loc += (*bodyIt)->m_r_S;

                // Binet Tensor (with reference to Interial System I) calculate only the 6 essential values
                #define r_IS  (*bodyIt)->m_r_S
                // m_Theta_I_loc += r_IS.transpose() * r_IS; (symmetric)
                m_Theta_I_loc(0) += r_IS(0)*r_IS(0);
                m_Theta_I_loc(1) += r_IS(0)*r_IS(1);
                m_Theta_I_loc(2) += r_IS(0)*r_IS(2);
                m_Theta_I_loc(3) += r_IS(1)*r_IS(1);
                m_Theta_I_loc(4) += r_IS(1)*r_IS(2);
                m_Theta_I_loc(5) += r_IS(2)*r_IS(2);
                #undef r_IS
            }

            m_r_G_loc /= this->m_pDynSys->m_SimBodies.size();
            // Move inertia tensor to geometric center (steiner formula) on root!

             // Predict Bodies over some steps
            // Either use Fluidix to quickly approximate a simulation or do a simple integration with only gravity of all point masses, stop if collided with non simulated bodies
        }

        //only master rank
        void buildGrid(){

            //Calculate principal axis of Theta_G
            Matrix33 Theta_G;
            Utilities::setSymMatrix(Theta_G,m_theta_G_glo);
            EigenSolverSelfAdjoint<Matrix33> eigenSolv(Theta_G);
            //LOG( eigenSolv.eigenvalues();
            eigenSolv.eigenvectors();

            // Expand grid by some percentage
            PREC dmax = m_aabb_glo.maxExtent();
            ASSERTMSG(dmax>=0,"max extend not >=0");
            //m_totalAABB.expand(dmax*0.)

        }


     private:

        Vector3 m_r_G_loc;       ///< Geometric center of all point masses
        Vector6 m_Theta_I_loc;  ///< Intertia tensor (classical, not the binet (euler) inertia tensor)
        AABB m_aabb_loc;         ///< Local AABB of point masses

        // GLOBAL STUFF =================================================================
        struct MassPoint{
            std::vector<Vector3> m_points; //< For the prediction
            RigidBodyState m_state; // Init Condition of the MassPoint
        };

        typename std::unordered_map<RigidBodyIdType, RigidBodyState> m_initStates;
        typename std::unordered_map<RankIdType,std::vector<RigidBodyIdType> m_bodiesPerRank;

        AABB m_aabb_glo;         ///< Global AABB of point masses
        Vector3 m_r_G__glo;      ///< Global geometric center of all point masses
        Vector6 m_theta_G_glo;  ///< Global intertia tensor (binet (euler) inertia tensor [a_00,a_01,a_02,a_11,a_12,a_22] )

        std::unordered_map<RankIdType,AABB> m_rankAABBs;

        std::unordered_map<RigidBodyIdType,MassPoint> m_massPoints_glo; ///< local mass points

        unsigned int m_nGlobalSimBodies; ///< Only for a check with MassPoints
};

}; //MPILayer

#endif


#ifndef GRSF_Dynamics_General_MPITopologyVisitors_hpp
#define GRSF_Dynamics_General_MPITopologyVisitors_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

//#include "GRSF/Dynamics/General/MPITopologyGrid.hpp"



namespace MPILayer {

template <typename ProcessTopologyBase> class ProcessTopologyGrid;

namespace TopologyVisitors {



    template<typename ProcessTopologyBase>
    class BelongsPointToProcess : public boost::static_visitor<bool> {
        public:
            DEFINE_MATRIX_TYPES
            DEFINE_MPI_INFORMATION_CONFIG_TYPES

            BelongsPointToProcess(const Vector3 & point, RankIdType &neighbourProcessRank)
                : m_point(point), m_neighbourProcessRank(neighbourProcessRank)
            {}

            // Implementation for Grid
            inline bool operator()(const MPILayer::ProcessTopologyGrid<ProcessTopologyBase> & topo) {
                m_neighbourProcessRank = topo.getCellRank(m_point);
                if(m_neighbourProcessRank == topo.m_rank) {
                    return true;
                }
                return false;
            }
            inline bool operator()(const boost::blank & b) {
                ASSERTMSG(false,"Topo is not initialized!");
            }
        private:
            const Vector3 & m_point;
            //Results
            RankIdType & m_neighbourProcessRank;
    };

    template<typename ProcessTopologyBase>
    class CheckOverlap : public boost::static_visitor<bool> {
    public:
        CheckOverlap( const typename ProcessTopologyBase::RigidBodyType * body,
                      typename ProcessTopologyBase::NeighbourRanksListType & neighbourProcessRanks,
                      bool & overlapsOwnProcess)
            : m_body(body),m_neighbourProcessRanks(neighbourProcessRanks), m_overlapsOwnProcess(overlapsOwnProcess)
        {}

        // Implementation for Grid
        inline bool operator()(MPILayer::ProcessTopologyGrid<ProcessTopologyBase> & topo) {
            m_neighbourProcessRanks.clear();
            return topo.checkOverlap(m_body,m_neighbourProcessRanks,m_overlapsOwnProcess);
        }

        inline bool operator()(const boost::blank & b) {
            ASSERTMSG(false,"Topo is not initialized!");
        }
    private:
        const typename ProcessTopologyBase::RigidBodyType * m_body;
        //Results
        typename ProcessTopologyBase::NeighbourRanksListType & m_neighbourProcessRanks;
        bool & m_overlapsOwnProcess;
    };

//    template<typename ProcessTopologyBase>
//    class Deleter: public boost::static_visitor<> {
//        public:
//            inline void operator()(ProcessTopologyGrid<ProcessTopologyBase> & topo) const {
//                ASSERTMSG(topo,"Topo pointer is zero?");
//                delete topo;
//            }
//            inline void operator()(boost::blank & b) const{
//                // no entry defined in the variant, nothing to delete!
//            }
//    };

}; // TopologyVisitors
}; // MPILayer
#endif

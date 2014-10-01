#ifndef MPITopologyVisitors_hpp
#define MPITopologyVisitors_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

//#include "MPITopologyGrid.hpp"



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
            // Check neighbour AABB
            m_neighbourProcessRanks.clear();
            for(auto it = topo.m_nbAABB.begin(); it != topo.m_nbAABB.end(); it++) {
                if( topo.m_Collider.checkOverlap(m_body,it->second) ) {
                    m_neighbourProcessRanks.insert(it->first);
                }
            }

            // Check own AABB
            m_overlapsOwnProcess = topo.m_Collider.checkOverlap(m_body, topo.m_aabb);

            return m_neighbourProcessRanks.size() > 0;
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

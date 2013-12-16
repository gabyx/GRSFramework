#ifndef ContactGraphNodeDataMPI_hpp
#define ContactGraphNodeDataMPI_hpp


#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "ContactGraphNodeData.hpp"

class ContactGraphNodeDataSplitBody{
    public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataSplitBody(RigidBodyType * body): m_body(body){};

    ~ContactGraphNodeDataSplitBody(){};

    bool addRank(const RankIdType & rank){
            auto pairRes = m_partRanks.insert(rank);
            return pairRes.second;
    }

   unsigned int getMultiplicity(){ return m_partRanks.size();}

    private:
        std::set<RankIdType> m_partRanks; ///< Participating ranks, defines the multiplicity
        RigidBodyType * m_body;
};


#endif

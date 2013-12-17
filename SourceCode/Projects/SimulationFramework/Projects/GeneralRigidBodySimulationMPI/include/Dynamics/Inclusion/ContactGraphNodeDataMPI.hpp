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
            auto pairRes = m_partRanks.push_back(rank);
            return pairRes.second;
    }

   /** plus one because this rank counts also as on split body to the overall body*/
   unsigned int getMultiplicity(){ return m_partRanks.size() + 1;}

    private:
        std::vector<RankIdType> m_partRanks; ///< Participating ranks, defines the multiplicity
        RigidBodyType * m_body;

        VectorDyn m_u_G; // all velocities of all split bodies m_u_G = [u1,u2,u3,u4,u5...], correspond to rank in vector
        VectorDyn m_lambdaB_back; //Bilateral Lambda
        VectorDyn m_lambdaB_front; //Bilateral Lambda
};


#endif

#ifndef ContactGraphNodeDataMPI_hpp
#define ContactGraphNodeDataMPI_hpp

#include <vector>
#include <map>

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "ContactGraphNodeData.hpp"

class ContactGraphNodeDataSplitBody {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataSplitBody(RigidBodyType * body): m_pBody(body),m_nLambdas(0) {

    };

    ~ContactGraphNodeDataSplitBody() {};

    bool addRank(const RankIdType & rank) {

        auto pairRes = m_partRanks.insert( std::make_pair(rank,m_partRanks.size()) );

        m_nLambdas +=1; //Increase lambda by 1 (one more constraint)

        return pairRes.second;
    }

    /** plus one because this rank counts also as on split body to the overall body*/
    inline unsigned int getMultiplicity() {
        return m_partRanks.size() + 1;
    }

    inline PREC getMultiplicityWeight(const RankIdType &rank){
        auto it = m_partRanks.find(rank);
        ASSERTMSG(it!=m_partRanks.end(), "Requested a weight for a non participating rank "<< rank << std::endl)
        return m_multiplicityWeights(it->second+1); // First weight belongs to local owner
    }

    std::map< RankIdType,unsigned int > m_partRanks; ///< Participating ranks to insertion number, defines the multiplicity

    /** Contains the weight factor for the partion of unity: [alpha_1, alpha_2, alpha_3,... , alpha_multiplicity]
    *   alpha_i = 1 / multiplicity so far, needs to be initialized befor the global prox loop */
    VectorDyn m_multiplicityWeights;





    RigidBodyType * m_pBody;

    /** These values get set from all remotes*/
    VectorDyn m_uBack;  ///                           Local Velocity-------*
    VectorDyn m_uFront; ///< all velocities of all split bodies m_u_G = [  u1 , u2,u3,u4,u5...], correspond to rank in vector

    //VectorDyn m_LambdaBack;  ///< Bilateral Lambda (Lambda_M_tilde = M⁻¹*Lambda_M
    VectorDyn m_LambdaFront; ///< Bilateral Lambda (Lambda_M_tilde = M⁻¹*Lambda_M
    unsigned int m_nLambdas; ///< How many bilateral constraints between bodies we have, currently = m_partRanks.size()

    VectorDyn m_gamma; ///< Gamma = [u_1-u_2, u_2-u_3, u_3-_u4,..., u_n-1 - u_n, ], u1 is always the velocity of the owner!

    inline MatrixSparse & getLInvMatrix(){
        auto mult = getMultiplicity();
        ASSERTMSG(mult>=2 && mult<=8, "Requested LMatrix for multiplicity: " << mult << " which is not implemented!");
        return m_LInvMatrices.LInv[mult];
    }



private:

    static struct LInvMatrices{
        LInvMatrices();
        std::vector<MatrixSparse> LInv;

    } m_LInvMatrices;

};


#endif

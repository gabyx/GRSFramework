#ifndef GRSF_Dynamics_Inclusion_ContactGraphNodeDataMPI_hpp
#define GRSF_Dynamics_Inclusion_ContactGraphNodeDataMPI_hpp

#include <vector>
#include <unordered_map>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"


#include "GRSF/Dynamics/General/RigidBodyId.hpp"

#include "GRSF/Dynamics/Inclusion/ContactGraphNodeData.hpp"


class ContactGraphNodeDataSplitBody {
public:

    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataSplitBody(){};
    ~ContactGraphNodeDataSplitBody(){};


    inline void initialize(RigidBodyType * body){
        m_pBody = body;
        m_nConstraints = 0;
        m_partRanks.clear();
    }

    inline void initData(){
        auto mult = getMultiplicity();

        // set all weights constant
        m_multiplicityWeights.setConstant(mult,1.0/mult);
        m_uBack.setZero(NDOFuBody,mult);
        m_uFront.setZero();
    }

    bool addRank(const RankIdType & rank) {

        auto pairRes = m_partRanks.emplace( rank,  Flags(m_partRanks.size()+1) );

        m_nConstraints +=1; //Increase lambda by 1 (one more constraint)

        return pairRes.second;
    }

    /** plus one because this rank counts also as one split body to the overall body*/
    inline unsigned int getMultiplicity() {
        return m_partRanks.size() + 1;
    }
    inline PREC getMultiplicityWeight(const RankIdType rank){
        auto it = m_partRanks.find(rank);
        ASSERTMSG(it!=m_partRanks.end(), "Requested a weight for a non participating rank "<< rank << std::endl)
        return m_multiplicityWeights(it->second.m_splitBodyIdx); // First weight belongs to local owner
    }
    inline void getMultiplicityAndWeight(const RankIdType rank, unsigned int & mult, PREC & multWeight){
        mult = getMultiplicity();
        auto it = m_partRanks.find(rank);
        ASSERTMSG(it!=m_partRanks.end(), "Requested a weight for a non participating rank "<< rank << std::endl);
        ASSERTMSG(m_multiplicityWeights.size()>0,"FUCK")
        multWeight = m_multiplicityWeights(it->second.m_splitBodyIdx); // First weight belongs to local owner
    }

    inline void updateVelocity(const RankIdType rank, const VectorUBody & u){
        auto it = m_partRanks.find(rank);
        ASSERTMSG(it!=m_partRanks.end(), "Rank: " << rank << " is not contained in the SplitBodyNode for body id: " << RigidBodyId::getBodyIdString(m_pBody));
        it->second.m_bGotUpdate = true;
        m_uBack.col(it->second.m_splitBodyIdx-1) = u;
    }

    /** m_splitBodyIdx is the internal number which is used in all subscripts in the comments in this class*/
    struct Flags{
        Flags(unsigned int splitBodyIdx): m_splitBodyIdx(splitBodyIdx), m_bGotUpdate(false){};
        const unsigned int m_splitBodyIdx;
        bool m_bGotUpdate;
    };
    std::unordered_map< RankIdType, Flags > m_partRanks; ///< Participating ranks Flags, size defines the multiplicity

    inline void resetFlags(){
        for(auto it = m_partRanks.begin(); it != m_partRanks.end(); it++){
            it->second.m_bGotUpdate = false;
        }
    }

    /** Contains the weight factor for the partion of unity: [alpha_0, alpha_1, alpha_2,... , alpha_multiplicity-1]
    *   alpha_i = 1 / multiplicity so far, needs to be initialized befor the global prox loop */
    VectorDyn m_multiplicityWeights;
    unsigned int m_nConstraints; ///< How many bilateral constraints between bodies we have, currently = m_partRanks.size()

    RigidBodyType * m_pBody = nullptr;

    /** These values get set from all remotes*/
    MatrixUBodyDyn   m_uBack; ///< all velocities of all split bodies m_u_G = [ u_1,u_2,u_3,u_4...], correspond to rank in vector
                              ///< local velocity is not contained
    VectorUBody      m_uFront; ///< The averages velocity of m_uBack

    inline MatrixSparse & getLInvMatrix(){
        auto mult = getMultiplicity();
        ASSERTMSG(mult>=2 && mult<=8, "Requested LMatrix for multiplicity: " << mult << " which is not implemented!");
        return m_LInvMatrices.LInv[mult-2];
    }


private:

    static struct LInvMatrices{
        LInvMatrices();
        std::vector<MatrixSparse> LInv;

    } m_LInvMatrices;

};


#endif

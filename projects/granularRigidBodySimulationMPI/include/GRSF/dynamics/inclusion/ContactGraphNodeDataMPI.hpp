// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ContactGraphNodeDataMPI_hpp
#define GRSF_dynamics_inclusion_ContactGraphNodeDataMPI_hpp

#include <unordered_map>
#include <vector>

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/general/RigidBodyId.hpp"

#include "GRSF/dynamics/inclusion/ContactGraphNodeData.hpp"

class ContactGraphNodeDataSplitBody
{
    public:
    DEFINE_MPI_INFORMATION_CONFIG_TYPES
    DEFINE_RIGIDBODY_CONFIG_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactGraphNodeDataSplitBody(){};
    ~ContactGraphNodeDataSplitBody(){};

    inline void initialize(RigidBodyType* body)
    {
        m_pBody        = body;
        m_nConstraints = 0;
        m_partRanks.clear();
    }

    inline void initData()
    {
        auto mult = getMultiplicity();

        // set all weights by 1/multiplicity
        m_multiplicityWeights.setConstant(mult, 1.0 / mult);
        m_uBack.setZero(NDOFuBody, mult - 1);  // make space for all velocities
        m_uFront.setZero();
    }

    bool addRank(const RankIdType& rank)
    {
        auto pairRes = m_partRanks.emplace(rank, Flags(m_partRanks.size()));

        m_nConstraints += 1;  // Increase lambda by 1 (one more constraint)

        return pairRes.second;
    }

    /** plus one because this rank counts also as one split body to the overall body*/
    inline unsigned int getMultiplicity()
    {
        return m_partRanks.size() + 1;
    }
    inline PREC getMultiplicityWeight(const RankIdType rank)
    {
        auto it = m_partRanks.find(rank);
        GRSF_ASSERTMSG(it != m_partRanks.end(), "Requested a weight for a non participating rank " << rank << std::endl)
        return m_multiplicityWeights(it->second.m_splitBodyIdx + 1);  // First weight belongs to local owner
    }
    inline void getMultiplicityAndWeight(const RankIdType rank, unsigned int& mult, PREC& multWeight)
    {
        mult    = getMultiplicity();
        auto it = m_partRanks.find(rank);
        GRSF_ASSERTMSG(it != m_partRanks.end(),
                       "Requested a weight for a non participating rank " << rank << std::endl);
        GRSF_ASSERTMSG(m_multiplicityWeights.size() > 0, "FUCK")
        multWeight = m_multiplicityWeights(it->second.m_splitBodyIdx + 1);  // First weight belongs to local owner
    }

    inline void updateVelocity(const RankIdType rank, const VectorUBody& u)
    {
        auto it = m_partRanks.find(rank);
        GRSF_ASSERTMSG(it != m_partRanks.end(),
                       "Rank: " << rank << " is not contained in the SplitBodyNode for body id: "
                                << RigidBodyId::getBodyIdString(m_pBody));
        it->second.m_bGotUpdate                = true;
        m_uBack.col(it->second.m_splitBodyIdx) = u;
    }

    inline void resetFlags()
    {
        for (auto& p : m_partRanks)
        {
            p.second.m_bGotUpdate = false;
        }
    }

    struct Flags
    {
        Flags(unsigned int splitBodyIdx) : m_splitBodyIdx(splitBodyIdx), m_bGotUpdate(false){};
        const unsigned int m_splitBodyIdx;  ///< index in the range [0,getMultiplicity()-2] into m_uBack
        bool               m_bGotUpdate;
    };
    std::unordered_map<RankIdType, Flags>
        m_partRanks;  ///< Flags for all participating ranks, size defines the multiplicity

    RigidBodyType* m_pBody = nullptr;

    /** velocities of all split bodies m_u_G = [ u_1,u_2,u_3,u_4...], local velocity u_0 is not contained */
    MatrixUBodyDyn m_uBack;
    VectorUBody    m_uFront;  ///< The affinely combined velocity of m_uBack

    /** Contains the weight factor for the partition of unity: [alpha_0, alpha_1, alpha_2,... , alpha_multiplicity-1]
    *   alpha_i = 1 / multiplicity so far, needs to be initialized before the global prox loop */
    VectorDyn m_multiplicityWeights;
    unsigned int
        m_nConstraints;  ///< How many bilateral constraints between bodies we have, currently = m_partRanks.size()

    //    inline MatrixSparse & getLInvMatrix(){
    //        auto mult = getMultiplicity();
    //        GRSF_ASSERTMSG(mult>=2 && mult<=8, "Requested LMatrix for multiplicity: " << mult << " which is not
    //        implemented!");
    //        return m_LInvMatrices.LInv[mult-2];
    //    }

    private:
    /** No more used, LInv matrix from derivation */
    static struct LInvMatrices
    {
        LInvMatrices();
        std::vector<MatrixSparse> LInv;

    } m_LInvMatrices;
};

#endif

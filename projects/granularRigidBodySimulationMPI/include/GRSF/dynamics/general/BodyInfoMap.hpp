// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_BodyInfoMap_hpp
#define GRSF_dynamics_general_BodyInfoMap_hpp

#include "GRSF/common/TypeDefs.hpp"

class BodyProcessInfo
{
public:
    DEFINE_RIGIDBODY_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    BodyProcessInfo(RankIdType ownRank,
                    bool overlapsThisRank = true,
                    bool isRemote         = false,
                    bool receivedUpdate   = false)
        : m_ownerRank(ownRank)
        , m_overlapsThisRank(overlapsThisRank)
        , m_isRemote(isRemote)
        , m_receivedUpdate(receivedUpdate){};
    /**
        * Data structure in the Map: Rank -> Flags, Flags define the behaviour what needs to be done with this Body.
        * m_overlaps: Used to decide if body is removed from the corresponding neigbourStructure
        */
    struct Flags
    {
        Flags(bool overlap = true, bool inNeighbourMap = true)
            : m_overlaps(overlap), m_inNeighbourMap(inNeighbourMap){};
        bool m_overlaps;        ///< If this body overlaps this neighbour in this timestep
        bool m_inNeighbourMap;  ///< If this body is contained in the neighbourmap or not!
    };

    using RankToFlagsType = std::unordered_map<RankIdType, Flags>;
    RankToFlagsType m_neighbourRanks;  ///< if body is remote: only one rankId has m_inNeighbourMap= true (only in the
                                       /// neighbour data it belongs to)

    RankIdType m_ownerRank;   ///< The process rank to which this body belongs (changes during simulation, if change ->
                              /// send to other process)
    bool m_overlapsThisRank;  ///< True if body overlaps this process!, if false

    bool m_isRemote;
    bool m_receivedUpdate;

    void resetNeighbourFlags()
    {
        for (auto it = m_neighbourRanks.begin(); it != m_neighbourRanks.end();)
        {
            it->second.m_overlaps = false;
            if (it->second.m_inNeighbourMap == false)
            {
                it = m_neighbourRanks.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    bool markNeighbourRankToRemove(RankIdType rank)
    {
        auto it = m_neighbourRanks.find(rank);
        if (it != m_neighbourRanks.end())
        {
            it->second.m_inNeighbourMap = false;
            return true;
        }
        return false;
    }
};

#endif

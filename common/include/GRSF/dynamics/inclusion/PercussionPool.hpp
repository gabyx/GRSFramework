// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_PercussionPool_hpp
#define GRSF_dynamics_inclusion_PercussionPool_hpp

#include <unordered_map>
#include <vector>

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/dynamics/collision/ContactTag.hpp"
#include "GRSF/dynamics/inclusion/ContactPercussion.hpp"

struct PercussionPoolSettings
{
    unsigned int m_cacheUnusedMaxSteps = 1;  ///< Amount of times the unused percussions are cached
};

/**
* @ingroup Inclusion
* @brief Percussion Pool.
*/
class PercussionPool
{
    public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using PercussionMap = std::unordered_map<ContactTag, ContactPercussion*, ContactTagHash>;

    PercussionPool(const PercussionPoolSettings& settings = PercussionPoolSettings())
        : m_settings(settings), m_usedPercussions(0)
    {
    }

    ~PercussionPool()
    {
        for (auto& s : m_percussionMap)
        {
            delete s.second;
        }
    }

    void reset(unsigned int counter = 0)
    {
        m_percussionMap.clear();
        m_usedPercussions = 0;
    }

    void rehash(unsigned int n)
    {
        // pointers and references stay valid!
        m_percussionMap.rehash(n);
    }

    void cleanUpAfterTimeStep(unsigned int counter = 0)
    {
        if (counter % m_settings.m_cacheUnusedMaxSteps == 0)
        {
            resetPercussions<true>();
        }
        else
        {
            resetPercussions<false>();
        }
    }

    /** Gets a percussion if this pool provides one, if forceCache is true then if the percussion does not exist one is
    * created!
    * @return Pointer to a percussion (nullptr or valid) and a bool indicating if this percussion can be used to
    * initialize!
    */
    template <bool forceCache = true>
    inline std::pair<ContactPercussion*, bool> getPercussion(const ContactTag& tag, unsigned int dim)
    {
        // See if there is a percussion in the cache otherwise insert an empty pointer
        ContactPercussion*& p = m_percussionMap.emplace(tag, nullptr).first->second;

        if (p == nullptr)
        {  // no cached value we cannot use it

            if (forceCache)
            {
                // set the pointer in the map to a new percussion
                p = new ContactPercussion{dim, true};
                return std::make_pair(p, false);
            }
            else
            {
                return std::make_pair(nullptr, false);
            }
        }
        else
        {  // there is a cached value, see if we can use it
            if (p->m_used)
            {
                WARNINGMSG(false, "Percussion with tag already used!")
            }
            p->m_used = true;
            ++m_usedPercussions;
            return std::make_pair(p, true);
        }
    }

    std::size_t size()
    {
        return m_percussionMap.size();
    }

    std::pair<std::size_t, PREC> getUsage()
    {
        std::size_t s = m_percussionMap.size();
        return std::make_pair(s, s ? (double)m_usedPercussions / (double)s : 0);
    }

    private:
    template <bool removeUnused>
    void           resetPercussions()
    {
        auto it = m_percussionMap.begin();
        while (it != m_percussionMap.end())
        {
            ContactPercussion* p = it->second;
            if (p->m_used == false && removeUnused)
            {  // if not used in the last iteration and removal, remove it
                // delete percussion
                delete p;
                it = m_percussionMap.erase(it);
            }
            else
            {  // reset all to used to not used for next iteration
                p->m_used = false;
                it++;
            }
        }
        m_usedPercussions = 0;
    }

    PercussionPoolSettings m_settings;
    std::size_t            m_usedPercussions;  ///< Counter for used percussions
    PercussionMap          m_percussionMap;    ///< The percussion map!
};

/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
template <typename TContactGraph>
class LambdaInitLogic
{
    public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;

    LambdaInitLogic(PercussionPool* pool) : m_pool(pool)
    {
    }

    inline void reset()
    {
        m_updatedNodes = 0;
    }

    template <typename NodeDataType>
    inline void initLambda(NodeDataType& nodeData, unsigned int dim)
    {
        // Init LambdaBack
        bool initialized = false;
        if (m_pool)
        {
            auto pair = m_pool->getPercussion<true>(nodeData.m_pCollData->m_contactTag, dim);
            if (pair.second)
            {  // we can use this cache value!
                nodeData.m_LambdaBack = pair.first->m_Lambda;
            }
            initialized = pair.second;
        }

        // Default value for lambda
        if (!initialized)
        {
            nodeData.m_LambdaBack.setZero();
        }

        nodeData.m_LambdaFront.setZero();
    }

    private:
    unsigned int    m_updatedNodes = 0;
    PercussionPool* m_pool;
};

/**
@brief Visitor for class ContactGraph<TRigidBody,ContactGraphMode::ForIteration>
*/
template <typename TContactGraph>
class CachePercussionNodeVisitor
{
    public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType   = TContactGraph;
    using UCFNodeDataType    = typename ContactGraphType::UCFNodeDataType;
    using CommonEdgeDataType = typename ContactGraphType::CommonEdgeDataType;

    CachePercussionNodeVisitor(PercussionPool* pool) : m_pool(pool)
    {
    }

    template <typename TNode>
    inline void operator()(TNode& node)
    {
        auto& nodeData = node.getData();
        if (nodeData.m_cache)
        {
            nodeData.m_cache->m_Lambda = nodeData.m_LambdaBack;
        }
    }

    private:
    PercussionPool* m_pool;
};

#endif

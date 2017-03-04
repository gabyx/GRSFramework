// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_inclusion_ContactGraphVisitorsMPI_hpp
#define GRSF_dynamics_inclusion_ContactGraphVisitorsMPI_hpp

#include "GRSF/common/LogDefines.hpp"
#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/SimpleLogger.hpp"

#include "GRSF/dynamics/inclusion/ContactModels.hpp"
#include "GRSF/dynamics/inclusion/ProxFunctions.hpp"
#include InclusionSolverSettings_INCLUDE_FILE

#include "GRSF/dynamics/inclusion/ContactGraphVisitors.hpp"

/**
@brief Visitor for class ContactGraph
*/
template <typename TContactGraph>
class SorProxStepSplitNodeVisitor
{
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    SorProxStepSplitNodeVisitor(const InclusionSolverSettingsType& settings,
                                bool& globalConverged,
                                const unsigned int& globalIterationNeeded)
        : m_settings(settings), m_bConverged(globalConverged), m_globalIterationCounter(globalIterationNeeded)
    {
    }

    template <typename TNode>
    inline void operator()(TNode& node)
    {
        dispatch(node.getData());
    }

    void setLog(Logging::Log* solverLog)
    {
        m_pSolverLog = solverLog;
    }

    void dispatch(NodeDataType& node)
    {
        // Calculate the exact values for the bilateral split nodes

        LOGSLLEVEL3_CONTACT(
            m_pSolverLog,
            "---> SorProx, Billateral Node: =====================" << std::endl
                                                                   << "\t---> local body id: "
                                                                   << RigidBodyId::getBodyIdString(node.m_pBody->m_id)
                                                                   << std::endl);

        auto mult = node.getMultiplicity();
        LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> multiplicity: " << mult << std::endl;)

        // Simple version (affine combination of velocities) ===================================
        node.m_uFront = node.m_multiplicityWeights(0) * node.m_pBody->m_pSolverData->m_uBuffer.m_front;
        for (unsigned int i = 1; i < mult; ++i)
        {
            node.m_uFront += node.m_uBack.col(i - 1) * node.m_multiplicityWeights(i);
        }
        // ======================================================================================

        // LOGSLLEVEL3_CONTACT(m_pSolverLog, "\t---> nd.m_uFront: " << node.m_uFront.transpose() <<std::endl; );

        // Because we solve this bilateral contact directly, we are converged for this node!
        // no change of the flag m_bConverged

        // Copy local back
        node.m_pBody->m_pSolverData->m_uBuffer.m_front = node.m_uFront;
    }

private:
    Logging::Log* m_pSolverLog;
    const InclusionSolverSettingsType& m_settings;
    bool& m_bConverged;                            ///< Access to global flag for cancelation criteria
    const unsigned int& m_globalIterationCounter;  ///< Access to global iteration counter
};

/**
@brief Visitor for class ContactGraph
*/
template <typename TContactGraph>
class SorProxInitSplitNodeVisitor
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    template <typename TNode>
    void operator()(TNode& node)
    {
        dispatch(node.getData());
    }

    void dispatch(NodeDataType& nodeData)
    {
        nodeData.initData();
    }
};

/**
@brief Visitor for class ContactGraph
*/
template <typename TContactGraph>
class SplitNodeCheckUpdateVisitor
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES
    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    SplitNodeCheckUpdateVisitor()
    {
    }

    template <typename TNode>
    void operator()(TNode& node)
    {
        dispatch(node.getData());
    }

    inline void dispatch(NodeDataType& node)
    {
        for (auto it = node.m_partRanks.begin(); it != node.m_partRanks.end(); ++it)
        {
            if (it->second.m_bGotUpdate == false)
            {
                GRSF_ERRORMSG("Rank: " << it->first << " in SplitNode for body id: "
                                       << RigidBodyId::getBodyIdString(node.m_pBody)
                                       << " has not got update!");
            }
            else
            {
                it->second.m_bGotUpdate == false;
            }
        }
    }
};

#include "GRSF/dynamics/general/RigidBodyFunctionsMPI.hpp"

template <typename TContactGraph>
class SetWeightingLocalBodiesSplitNodeVisitor
{
public:
    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    SetWeightingLocalBodiesSplitNodeVisitor(){};

    template <typename TNode>
    inline void operator()(TNode& node)
    {
        dispatch(node.getData());
    }

    inline void dispatch(NodeDataType& node)
    {
        auto mult = node.getMultiplicity();
        RigidBodyFunctions::changeBodyToSplitWeighting(node.m_pBody, mult, node.m_multiplicityWeights(0));
    }
};

template <typename TContactGraph>
class ResetWeightingLocalBodiesSplitNodeVisitor
{
public:
    using ContactGraphType = TContactGraph;

    using NodeDataType = typename ContactGraphType::SplitBodyNodeDataType;

    ResetWeightingLocalBodiesSplitNodeVisitor(){};

    template <typename TNode>
    inline void operator()(TNode& node)
    {
        dispatch(node.getData());
    }

    inline void dispatch(NodeDataType& node)
    {
        RigidBodyFunctions::changeBodyToNormalWeighting(node.m_pBody);
    }
};

#endif  // ContactGraphVisitorMPI_hpp

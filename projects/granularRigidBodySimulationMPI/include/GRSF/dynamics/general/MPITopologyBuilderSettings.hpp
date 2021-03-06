// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPITopologyBuilderSettings_hpp
#define GRSF_dynamics_general_MPITopologyBuilderSettings_hpp

#include "GRSF/common/Asserts.hpp"
#include "GRSF/common/TypeDefs.hpp"

namespace MPILayer
{
enum class TopologyBuilderEnum : char
{
    GRIDBUILDER,
    KDTREEBUILDER
};

struct GridBuilderSettings
{
    DEFINE_LAYOUT_CONFIG_TYPES

    GridBuilderSettings() : m_processDim(MyMatrix::Array3<unsigned int>(1, 1, 1)), m_A_IK(Matrix33::Identity())
    {
    }

    using ProcessDimType = MyMatrix::Array3<unsigned int>;
    ProcessDimType m_processDim;

    bool m_matchProcessDimToExtent = true;

    /** Minimal grid dimension depends basically on the shapes simualated:
    * max_i diam(body_i) <= min gridSize_x, gridSize_y, gridSize_z $
    */
    PREC m_minCellSize = 1e-3;

    /**
    *   PREDEFINED: Take values from below!
    *   ALIGNED: fit AABB
    *   BINET_TENSOR: make OOBB in coordinate system of eigenvectors of the
                      BinetTensor (or principal componente analysis),
    *   MVBB: Minimum Volume Bounding Box
    */

    enum class BuildMode : short
    {
        PREDEFINED,
        ALIGNED,
        BINET_TENSOR,
        MVBB
    } m_buildMode = BuildMode::MVBB;

    // OOBB or AABB
    AABB3d m_aabb;  ///< used values for predefined values
    bool m_aligned = true;
    Matrix33 m_A_IK;
};

struct KdTreeBuilderSettings
{
    DEFINE_LAYOUT_CONFIG_TYPES

    KdTreeBuilderSettings() : m_A_IK(Matrix33::Identity())
    {
    }

    /** Minimal grid dimension depends basically on the shapes simualated:
    * max_i diam(body_i) <= min gridSize_x, gridSize_y, gridSize_z $
    */
    PREC m_minCellSize          = 1e-3;
    PREC m_minPointsForSplit    = 10;
    unsigned int m_maxTreeDepth = 5000;  ///< Huge such that this is not a constraint for stopping

    PREC m_minSplitRatio  = 0.0;
    PREC m_minPointRatio  = 0.0;
    PREC m_minExtentRatio = 0.0;

    /**
    *   PREDEFINED: Take values from below!
    *   ALIGNED: fit AABB
    *   BINET_TENSOR: make OOBB in coordinate system of eigenvectors of the
                      BinetTensor (or principal componente analysis),
    *   MVBB: Minimum Volume Bounding Box
    */

    enum class BuildMode : short
    {
        PREDEFINED,
        ALIGNED,
        BINET_TENSOR,
        MVBB
    } m_buildMode = BuildMode::MVBB;

    // OOBB or AABB
    AABB3d m_aabb;  ///< used values for predefined values
    bool m_aligned = true;
    Matrix33 m_A_IK;
};

class TopologyBuilderSettings
{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;

    /** Rebuilder Settings */
    struct RebuildSettings
    {
        enum class Mode : short
        {
            STATIC,
            DYNAMIC
        } m_mode = Mode::DYNAMIC;

        /** Dynamic Settings */
        unsigned int m_policyCheckEachXTimeStep = 10;
        enum class Policy
        {
            ALWAYS_REBUILD,
            BODY_LIMIT
        } m_policy = Policy::ALWAYS_REBUILD;

        // Body Limit Policy
        unsigned int m_bodyLimit = 500;

        /** Do local computations, if it is possible for the given topology computations
        *   For example: BINET_TENSOR, ALIGNED can compute the prediction and tensor locally and then assemble the
        *   total results on the master.
        *   For MVBB , local computations are not possible
        */
        bool m_doLocalComputations = true;
    };

    /** Mass point prediction */
    struct MassPointPredSettings
    {
        unsigned int m_nPoints = 5;
        PREC m_deltaT          = 0.1;
    };

    /** Outlier Filtering of Point Cloud*/
    struct OutlierFilterSettings
    {
        bool m_enabled                 = false;
        unsigned int m_kNNMean         = 20;
        unsigned int m_stdDevMult      = 4;
        unsigned int m_allowSplitAbove = 10;  // kdTree settings
    };

public:
    TopologyBuilderEnumType m_type;

    GridBuilderSettings m_gridBuilderSettings;
    KdTreeBuilderSettings m_kdTreeBuilderSettings;

    RebuildSettings m_rebuildSettings;

    /**   If this filter is enabled, m_rebuildSettings::m_doLocalComputations = false!
          during rebuilding.
    */
    OutlierFilterSettings m_globalOutlierFilterSettings;

    MassPointPredSettings m_massPointPredSettings;
};
};

#endif

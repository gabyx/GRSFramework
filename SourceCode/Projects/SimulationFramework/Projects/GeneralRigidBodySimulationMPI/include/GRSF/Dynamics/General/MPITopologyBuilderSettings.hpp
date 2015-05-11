#ifndef GRSF_Dynamics_General_MPITopologyBuilderSettings_hpp
#define GRSF_Dynamics_General_MPITopologyBuilderSettings_hpp

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/TypeDefs.hpp"

namespace MPILayer{



enum class TopologyBuilderEnum : char{
    GRIDBUILDER
    /*SuperDuperHexagonBuilder*/
};


struct GridBuilderSettings{
    DEFINE_LAYOUT_CONFIG_TYPES

    GridBuilderSettings(): m_processDim(MyMatrix<unsigned int>::Array3(1,1,1)), m_A_IK(Matrix33::Identity()){}

    using ProcessDimType = MyMatrix<unsigned int>::Array3;
    ProcessDimType m_processDim;

    bool m_matchProcessDimToExtent = true;

    /** Minimal grid dimension depends basically on the shapes simualated:
    * max_i diam(body_i) <= min gridSize_x, gridSize_y, gridSize_z $
    */
    PREC m_minGridSize = 1e-3;


    /**
    *   PREDEFINED: Take values from below!
    *   ALIGNED: fit AABB
    *   BINET_TENSOR: fit to frame of Eigenvalue decomposition of BinetTensor,
    *   MVBB: Minimum Volume Bounding Box
    */

    enum class BuildMode : short{ PREDEFINED , ALIGNED, BINET_TENSOR, MVBB} m_buildMode = BuildMode::MVBB;

    // OOBB or AABB
    AABB3d m_aabb; ///< used values for predefined values
    bool m_aligned = true;
    Matrix33 m_A_IK;
};



struct KdTreeBuilderSettings{
    DEFINE_LAYOUT_CONFIG_TYPES

    KdTreeBuilderSettings(): m_A_IK(Matrix33::Identity()){}

    unsigned int m_processes = 1;

    /** Minimal grid dimension depends basically on the shapes simualated:
    * max_i diam(body_i) <= min gridSize_x, gridSize_y, gridSize_z $
    */
    PREC m_minCellSize = 1e-3;
    PREC m_minPointsForSplit = 10;
    unsigned int m_maxTreeDepth = 5000; ///< Huge such that this is not a constraint for stopping
    /**
    *   PREDEFINED: Take values from below!
    *   ALIGNED: fit AABB
    *   MVBB: Minimum Volume Bounding Box
    */

    enum class BuildMode : short{ PREDEFINED , ALIGNED, MVBB} m_buildMode = BuildMode::MVBB;

    // OOBB or AABB
    AABB3d m_aabb; ///< used values for predefined values
    bool m_aligned = true;
    Matrix33 m_A_IK;
};



class TopologyBuilderSettings{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;

//
//    inline TopologyBuilderEnumType & type(){ return m_type;}
//    inline const TopologyBuilderEnumType & type() const { return m_type;}
//
//    inline GridBuilderSettings & gridBuilderSettings(){
//        return m_gridBuilderSettings;
//    }
//    inline const GridBuilderSettings & gridBuilderSettings() const {
//        return m_gridBuilderSettings;
//    }
//
//    inline KdTreeBuilderSettings & kdTreeBuilderSettings(){
//        return m_kdTreeBuilderSettings;
//    }
//    inline const KdTreeBuilderSettings & kdTreeBuilderSettings() const {
//        return m_kdTreeBuilderSettings;
//    }
//


    struct RebuildSettings{

        enum class Mode : short{ STATIC, DYNAMIC} m_mode = Mode::DYNAMIC;

        /** Dynamic Settings */
        unsigned int m_policyCheckEachXTimeStep = 10;
        enum class Policy{ ALWAYS_REBUILD, BODY_LIMIT } m_policy = Policy::ALWAYS_REBUILD;

        // Body Limit Policy
        unsigned int m_bodyLimit = 500;

    };


    struct MassPointPredSettings{
        MassPointPredSettings(): m_deltaT(0.1){}
        unsigned int m_nPoints = 5;
        PREC m_deltaT;
    };

//    inline RebuildSettings & rebuildSettings(){ return m_rebuildSettings;}
//    inline const RebuildSettings & rebuildSettings() const { return m_rebuildSettings;}
//
//    inline MassPointPredSettings & massPointPredSettings(){ return m_massPointPredSettings;}
//    inline const MassPointPredSettings & massPointPredSettings() const { return m_massPointPredSettings;}

public:

    TopologyBuilderEnumType m_type;

    GridBuilderSettings    m_gridBuilderSettings;
    KdTreeBuilderSettings  m_kdTreeBuilderSettings;

    RebuildSettings m_rebuildSettings;
    MassPointPredSettings m_massPointPredSettings;

};

};


#endif

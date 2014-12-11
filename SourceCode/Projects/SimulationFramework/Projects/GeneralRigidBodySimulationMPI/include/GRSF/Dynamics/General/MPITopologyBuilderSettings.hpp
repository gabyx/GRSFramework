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
    GridBuilderSettings(): m_processDim(MyMatrix<unsigned int>::Array3(1,1,1)){
        m_aabb.reset();
        m_A_IK = Matrix33::Identity();
    }
    using ProcessDimType = MyMatrix<unsigned int>::Array3;
    ProcessDimType m_processDim;

    enum class Mode : short{ STATIC, DYNAMIC} m_mode = Mode::DYNAMIC;

    /**
    *   PREDEFINED: Take values from below!
    *   ALIGNED: fit AABB
    *   BINET_TENSOR: fit to frame of Eigenvalue decomposition of BinetTensor,
    *   MVBB: Minimum Volume Bounding Box
    */

    enum class BuildMode : short{ PREDEFINED , ALIGNED, BINET_TENSOR, MVBB} m_buildMode = BuildMode::MVBB;

    // OOBB or AABB
    AABB m_aabb; ///< used values for predefined values
    bool m_aligned = true;
    Matrix33 m_A_IK;
};


class TopologyBuilderSettings{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;
    TopologyBuilderEnumType m_type;


    GridBuilderSettings  m_gridBuilderSettings;

    struct RebuildSettings{
        unsigned int m_rebuildingCheckEachXTimeStep = 10;
        enum class Policy{ NOTHING, BODY_LIMIT } m_policy = Policy::NOTHING;

        // Body Limit Policy
        unsigned int m_bodyLimit = 500;

    } m_rebuildSettings;

};

};


#endif

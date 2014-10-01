#ifndef MPITopologyBuilderSettings_hpp
#define MPITopologyBuilderSettings_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

namespace MPILayer{


enum class TopologyBuilderEnum : char{
    GRIDBUILDER
    /*SuperDuperHexagonBuilder*/
};


struct GridBuilderSettings{
    DEFINE_LAYOUT_CONFIG_TYPES
    GridBuilderSettings(): m_processDim(MyMatrix<unsigned int>::Vector3(1,1,1)){
        m_minPoint.setZero();
        m_maxPoint.setZero();
    }
    MyMatrix<unsigned int>::Vector3 m_processDim;
    enum class Mode : short{ STATIC, DYNAMIC} m_mode = Mode::DYNAMIC;
    Vector3 m_minPoint, m_maxPoint;
};


class TopologyBuilderSettings{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;
    TopologyBuilderEnumType m_type;


    GridBuilderSettings  m_gridBuilderSettings;

};

};


#endif

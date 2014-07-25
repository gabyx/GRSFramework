#ifndef MPITopologyBuilderSettings_hpp
#define MPITopologyBuilderSettings_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

namespace MPILayer{

enum class TopologyBuilderEnum : char{
    GRIDBUILDER
    /*SuperDuperHexagonBuilder*/
};

class TopologyBuilderSettings{
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    using TopologyBuilderEnumType = TopologyBuilderEnum;
    TopologyBuilderEnumType m_type;

    //For GridBuilder
    MyMatrix<unsigned int>::Vector3 m_processDim;

};

};


#endif

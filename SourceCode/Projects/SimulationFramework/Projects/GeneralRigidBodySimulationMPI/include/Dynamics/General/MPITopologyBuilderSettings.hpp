#ifndef MPITopologyBuilderSettings_hpp
#define MPITopologyBuilderSettings_hpp

#include "AssertionDebug.hpp"
#include "TypeDefs.hpp"

namespace MPILayer{

enum class: char TopologyBuilderType{
    GRIDBUILDER
    /*SuperDuperHexagonBuilder*/
};

class TopologyBuilderSettings{

    DEFINE_LAYOUT_CONFIG_TYPES

    TopologyBuilderType m_type;

    //For GridBuilder
    MyMatrix<unsigned int>::Vector3 m_processDimensions;

};

};


#endif

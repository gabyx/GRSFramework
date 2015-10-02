#ifndef GRSF_General_GridExtractionSettings_hpp
#define GRSF_General_GridExtractionSettings_hpp

#include <string>
#include <vector>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/LogDefines.hpp"

#include "GRSF/Dynamics/Collision/Geometry/AABB.hpp"


struct GridExtractionSettings{
public:

    DEFINE_MATRIX_TYPES

    using Array3Int = typename MyMatrix<std::size_t>::Array3;

    std::string m_fileName;
    AABB3d m_aabb;
    Matrix33 m_R_KI;
    Array3Int m_dimension;

};


#endif

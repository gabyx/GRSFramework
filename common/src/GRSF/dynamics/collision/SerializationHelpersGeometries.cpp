#include "GRSF/dynamics/collision/SerializationHelpersGeometries.hpp"


template<>
void GeomSerialization::createGeom_impl<-1>(){
    ERRORMSG("Geometry which type: " << m_w << "did not match with the variant!");
}

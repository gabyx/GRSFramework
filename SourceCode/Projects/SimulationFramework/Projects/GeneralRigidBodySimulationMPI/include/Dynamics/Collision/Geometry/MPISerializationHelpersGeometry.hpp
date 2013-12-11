#ifndef MPISerializationHelpersGeometry_hpp
#define MPISerializationHelpersGeometry_hpp

#include <Eigen/Dense>
#include <boost/serialization/array.hpp>

#include "BoxGeometry.hpp"
#include "SphereGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "MeshGeometry.hpp"

namespace boost {
namespace serialization {

    template<class Archive>
    void serialize(Archive & ar, BoxGeometry & g, const unsigned int version) {
        serializeEigen(ar,g.m_extent);
        serializeEigen(ar,g.m_center);
    }

    template<class Archive>
    void serialize(Archive & ar, HalfspaceGeometry & g, const unsigned int version) {

        serializeEigen(ar,g.m_normal);
        serializeEigen(ar,g.m_pos);
    }

    template<class Archive>
    void serialize(Archive & ar, SphereGeometry & g, const unsigned int version) {

        ar & g.m_radius;

    }

    template<class Archive>
    void serialize(Archive & ar, MeshGeometry & g, const unsigned int version) {
        ERRORMSG("No implementation for MeshGeometry serialization!");
    }

};
};

#endif

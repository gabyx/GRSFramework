#ifndef MPISerializationHelpersGeometry_hpp
#define MPISerializationHelpersGeometry_hpp

#include <boost/variant.hpp>

#include "BoxGeometry.hpp"
#include "SphereGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "MeshGeometry.hpp"
#include "AABB.hpp"

#include "MPISerializationHelpersEigen.hpp"


template<typename Archive>
class GeomVisitorSerialization: public boost::static_visitor<> {
private:
    Archive & m_ar;
public:
    GeomVisitorSerialization(Archive & ar): m_ar(ar) {}
    void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom)  {
        ar & *sphereGeom;
    }

    void operator()(std::shared_ptr<const BoxGeometry > & box)  {
        ar & *box;
    }

    void operator()(std::shared_ptr<const MeshGeometry > & mesh)  {
        ar & *mesh;
    }

    void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
        ar & *halfspace;
    }
};


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
void serialize(Archive & ar, AABB & g, const unsigned int version) {

    serializeEigen(ar & g.m_minPoint);
    serializeEigen(ar & g.m_maxPoint);

}


template<class Archive>
void serialize(Archive & ar, MeshGeometry & g, const unsigned int version) {

    ERRORMSG("No implementation for MeshGeometry serialization!");
}

};
};

#endif

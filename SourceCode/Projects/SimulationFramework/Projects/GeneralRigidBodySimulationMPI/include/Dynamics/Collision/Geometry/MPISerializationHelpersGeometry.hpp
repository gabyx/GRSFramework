#ifndef MPISerializationHelpersGeometry_hpp
#define MPISerializationHelpersGeometry_hpp

#include <boost/variant.hpp>

#include "BoxGeometry.hpp"
#include "SphereGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "MeshGeometry.hpp"
#include "AABB.hpp"

#include "MPISerializationHelpersEigen.hpp"

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




template<typename Archive, typename WhichType, typename Variant>
class GeomVisitorSerialization: public boost::static_visitor<> {
private:
    Archive & m_ar;
    WhichType m_w;
    Variant & m_v;
public:
    GeomVisitorSerialization(Archive & ar, WhichType w, Variant v): m_ar(ar), m_w(w), m_v(v) {
//        if(! Archive::is_saving::value ){
//            if(w == boost::mpl::at){
//            case
//            }
//        }
    }

    void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom)  {
        if(Archive::is_saving::value){
            m_ar & *sphereGeom;
        }else{

        }
    }

    void operator()(std::shared_ptr<const BoxGeometry > & box)  {
        m_ar & *box;
    }

    void operator()(std::shared_ptr<const MeshGeometry > & mesh)  {
        m_ar & *mesh;
    }

    void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
        m_ar & *halfspace;
    }
};




#endif

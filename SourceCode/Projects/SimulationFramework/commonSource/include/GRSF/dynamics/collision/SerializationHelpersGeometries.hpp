#ifndef GRSF_Dynamics_Collision_SerializationHelpersGeometries_hpp
#define GRSF_Dynamics_Collision_SerializationHelpersGeometries_hpp

#include <boost/variant.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/mpl/at.hpp>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/StaticAssert.hpp"

#include RigidBody_INCLUDE_FILE

#include "GRSF/Dynamics/Collision/Geometries.hpp"

#include "GRSF/Common/SerializationHelpersEigen.hpp"

namespace boost {
namespace serialization {

template<typename Archive>
void serialize(Archive & ar, BoxGeometry & g, const unsigned int version) {
    serializeEigen(ar,g.m_extent);
    serializeEigen(ar,g.m_center);
}

template<typename Archive>
void serialize(Archive & ar, HalfspaceGeometry & g, const unsigned int version) {

    serializeEigen(ar,g.m_normal);
    /*serializeEigen(ar,g.m_pos);*/
}

template<typename Archive>
void serialize(Archive & ar, SphereGeometry & g, const unsigned int version) {

    ar & g.m_radius;

}

template<typename Archive>
void serialize(Archive & ar, CapsuleGeometry & g, const unsigned int version) {

    ar & g.m_radius;
    ar & g.m_length;
    serializeEigen(ar,g.m_normal);
}

template<typename Archive, unsigned int N>
void serialize(Archive & ar, AABB<N> & g, const unsigned int version) {

    serializeEigen(ar , g.m_minPoint);
    serializeEigen(ar , g.m_maxPoint);

}


template<typename Archive>
void serialize(Archive & ar, MeshGeometry & g, const unsigned int version) {

    ERRORMSG("No implementation for MeshGeometry serialization!");

}

};
};




class GeomSerialization{
private:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)

    using GeometryType = typename RigidBodyType::GeometryType;
    using VariantTypes = typename GeometryType::types;

    GeometryType & m_g;

    typedef decltype(m_g.which()) WhichType;
    WhichType m_w;

    template<int N>
    void createGeom_impl(){

        using SharedPtrType = typename boost::mpl::at_c<VariantTypes, N>::type;
        using GeomType = typename SharedPtrType::element_type;

        if(N == m_w){
            m_g = SharedPtrType( new GeomType() ); // replaces the shared_ptr which destructs the hold object in m_g
        }else{
            createGeom_impl<N-1>();
        }
    }
    void createGeom(){
         //Recursive template
         createGeom_impl< boost::mpl::size<VariantTypes>::value - 1 >();
    }
    template<typename Archive>
    struct GeomVis: public boost::static_visitor<>{
        GeomVis(Archive & ar): m_ar(ar){};
        inline void operator()(SphereGeomPtrType & sphereGeom)  {
            m_ar & const_cast<SphereGeometry&>(*sphereGeom);
        }
        inline void operator()(std::shared_ptr<const BoxGeometry > & box)  {
            m_ar & const_cast<BoxGeometry&>(*box);
        }
        inline void operator()(std::shared_ptr<const MeshGeometry > & mesh)  {
            m_ar & const_cast<MeshGeometry&>(*mesh);
        }
        inline void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            m_ar & const_cast<HalfspaceGeometry&>(*halfspace);
        }
        inline void operator()(std::shared_ptr<const CapsuleGeometry > & capsule)  {
            m_ar & const_cast<CapsuleGeometry&>(*capsule);
        }
        Archive & m_ar;
    };

public:
    GeomSerialization(GeometryType & g): m_g(g) {
        m_w=m_g.which();
    }

    template<class Archive>
    void save(Archive & ar, const unsigned int version) const {
        ar << m_w;
        GeomVis<Archive> v(ar);
        m_g.apply_visitor(v);
    }
    template<class Archive>
    void load(Archive & ar, const unsigned int version){
        ar >> m_w;
        createGeom(); // make a new shared_ptr< GeomType >
        GeomVis<Archive> v(ar);
        m_g.apply_visitor(v);
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER();
};

template<> void GeomSerialization::createGeom_impl<-1>();


#endif

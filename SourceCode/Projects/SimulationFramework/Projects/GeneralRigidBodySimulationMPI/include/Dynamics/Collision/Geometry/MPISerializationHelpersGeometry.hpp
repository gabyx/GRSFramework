#ifndef MPISerializationHelpersGeometry_hpp
#define MPISerializationHelpersGeometry_hpp

#include <boost/variant.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/mpl/at.hpp>

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include RigidBody_INCLUDE_FILE

#include "AABB.hpp"
#include "SphereGeometry.hpp"
#include "PlaneGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "BoxGeometry.hpp"
#include "MeshGeometry.hpp"

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




class GeomSerialization{
private:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    typedef typename RigidBodyType::GeometryType GeometryType;
    typedef typename GeometryType::types VariantTypes;

    GeometryType & m_g;

    typedef decltype(m_g.which()) WhichType;
    WhichType m_w;

    template<int N>
    void createGeom_impl(){

        typedef typename boost::mpl::at_c<VariantTypes, N>::type SharedPtrType;
        typedef typename SharedPtrType::element_type GeomType;

        if(N == m_w){
            m_g = SharedPtrType( new GeomType() );
        }else{
            createGeom_impl<N-1>();
        }
    }
    void createGeom(){
         createGeom_impl< boost::mpl::size<VariantTypes>::value - 1 >();
    }
    template<typename Archive>
    struct GeomVis: public boost::static_visitor<>{
        GeomVis(Archive & ar): m_ar(ar){};
        void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom)  {
            m_ar & const_cast<SphereGeometry&>(*sphereGeom);
        }
        void operator()(std::shared_ptr<const BoxGeometry > & box)  {
            m_ar & const_cast<BoxGeometry&>(*box);
        }
        void operator()(std::shared_ptr<const MeshGeometry > & mesh)  {
            m_ar & const_cast<MeshGeometry&>(*mesh);
        }
        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            m_ar & const_cast<HalfspaceGeometry&>(*halfspace);
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

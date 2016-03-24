#ifndef GRSF_converters_renderer_RendermanGeometryWriter_hpp
#define GRSF_converters_renderer_RendermanGeometryWriter_hpp

#include <boost/variant.hpp>

#include "GRSF/common/TypeDefs.hpp"

#include RigidBody_INCLUDE_FILE

class RendermanGeometryWriter: public boost::static_visitor<> {
    public:

    DEFINE_RIGIDBODY_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)

    RendermanGeometryWriter(){}

    void setStream(std::stringstream * s){
        m_s = s;
    }

    inline void operator()(SphereGeomPtrType & sphereGeom) {

// Sphere
        *m_s << "Sphere " << sphereGeom->m_radius <<" "
        << -sphereGeom->m_radius  << " "
        << sphereGeom->m_radius  << " "
        << " 360"  <<"\n";

// Points
          //*m_s << "Points \"P\" [0 0 0]" << " \"width\" ["<<sphereGeom->m_radius <<"]\n";


    }

    template<typename T>
    inline void operator()(T & m)  {
        ERRORMSG("Not implemented")
    }


    private:
    std::stringstream * m_s;
};

#endif

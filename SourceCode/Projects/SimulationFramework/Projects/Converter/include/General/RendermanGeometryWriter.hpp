#ifndef RendermanGeometryWriter_hpp
#define RendermanGeometryWriter_hpp

#include <boost/variant.hpp>

#include "TypeDefs.hpp"

#include RigidBody_INCLUDE_FILE

template<
class RendermanGeometryWriter: public boost::static_visitor<> {
    public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    RendermanGeometryWriter(){}

    void setStream(std::stringstream * s){
        m_s = s;
    }

    inline void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom) {
        *m_s << "Sphere " << sphereGeom->m_radius <<" "
        << -sphereGeom->m_radius  << " "
        << sphereGeom->m_radius  << " "
        << " 360"  <<"\n";
    }

    inline void operator()(std::shared_ptr<const BoxGeometry > & box)  {
        ERRORMSG("Not implemented")
    }

    inline void operator()(std::shared_ptr<const MeshGeometry > & box) {
        ERRORMSG("Not implemented")
    }

    inline void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace) {
        ERRORMSG("Not implemented")
    }

    private:
    std::stringstream * m_s;
};

#endif

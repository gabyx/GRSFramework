#ifndef GRSF_Dynamics_General_AddGyroTermVisitor_hpp
#define GRSF_Dynamics_General_AddGyroTermVisitor_hpp

#include "GRSF/Common/TypeDefs.hpp"

#include RigidBody_INCLUDE_FILE

class AddGyroTermVisitor : public boost::static_visitor<> {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    AddGyroTermVisitor(RigidBodyType * body):
    m_rigidBody(body)
    {
        m_rigidBody->m_geometry.apply_visitor(*this);
    }


    inline void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom) {
        return;
    }

    inline void operator()(std::shared_ptr<const BoxGeometry > & box)  {
        addGyroTerm();
    }

    inline void operator()(std::shared_ptr<const MeshGeometry > & box) {
        addGyroTerm();
    }

    inline void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace) {
        addGyroTerm();
    }

    private:

    inline void addGyroTerm(){
        Vector3 K_omega_IK = m_rigidBody->m_pSolverData->m_uBuffer.m_back.tail<3>();
        m_rigidBody->m_h_term.tail<3>() -= K_omega_IK.cross((m_rigidBody->m_K_Theta_S.asDiagonal() * K_omega_IK).eval());
    }

    RigidBodyType * m_rigidBody;
};


#endif

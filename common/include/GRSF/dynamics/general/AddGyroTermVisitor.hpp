#ifndef GRSF_dynamics_general_AddGyroTermVisitor_hpp
#define GRSF_dynamics_general_AddGyroTermVisitor_hpp

#include "GRSF/common/TypeDefs.hpp"

#include RigidBody_INCLUDE_FILE


template<typename TRigidBody>
class AddGyroTermVisitor : public boost::static_visitor<> {
public:

    DEFINE_LAYOUT_CONFIG_TYPES
    using RigidBodyType = TRigidBody;
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)

    AddGyroTermVisitor(RigidBodyType * body):
    m_rigidBody(body)
    {
        m_rigidBody->m_geometry.apply_visitor(*this);
    }


    inline void operator()(SphereGeomPtrType & sphereGeom) {
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

    inline void operator()(std::shared_ptr<const CapsuleGeometry > & halfspace) {
        addGyroTerm();
    }

    private:

    inline void addGyroTerm(){
        Vector3 K_omega_IK = m_rigidBody->m_pSolverData->m_uBuffer.m_back.template tail<3>();
        m_rigidBody->m_h_term.template tail<3>() -= K_omega_IK.cross((m_rigidBody->m_K_Theta_S.asDiagonal() * K_omega_IK).eval());
    }

    RigidBodyType * m_rigidBody;
};


#endif

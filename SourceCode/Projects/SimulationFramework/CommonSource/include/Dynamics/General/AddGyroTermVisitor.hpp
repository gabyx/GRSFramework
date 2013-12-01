#ifndef AddGyroTermVisitor_hpp
#define AddGyroTermVisitor_hpp

#include <boost/variant.hpp>

#include "TypeDefs.hpp"



class AddGyroTermVisitor : public boost::static_visitor<> {
public:

    DEFINE_RIGIDBODY_CONFIG_TYPES

    AddGyroTermVisitor(RigidBodyType * body):
    m_rigidBody(body)
    {
        boost::apply_visitor(*this, m_rigidBody->m_geometry);
    }


    void operator()(boost::shared_ptr<const SphereGeometry<PREC> > & sphereGeom) {
        return;
    }

    void operator()(boost::shared_ptr<const BoxGeometry > & box)  {
        addGyroTerm();
    }

    void operator()(boost::shared_ptr<const MeshGeometry<PREC> > & box) {
        addGyroTerm();
    }

    void operator()(boost::shared_ptr<const HalfspaceGeometry > & halfspace) {
        addGyroTerm();
    }

    private:

    void addGyroTerm(){
        Vector3 K_omega_IK = m_rigidBody->m_pSolverData->m_uBuffer.m_back.tail<3>();
        m_rigidBody->m_h_term.tail<3>() -= K_omega_IK.cross((m_rigidBody->m_K_Theta_S.asDiagonal() * K_omega_IK).eval());
    }

    RigidBodyType * m_rigidBody;
};


#endif

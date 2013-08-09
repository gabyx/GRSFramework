#ifndef AddGyroTermVisitor_hpp
#define AddGyroTermVisitor_hpp

#include <boost/variant.hpp>

#include "TypeDefs.hpp"

template<typename TRigidBody>
class AddGyroTermVisitor : public boost::static_visitor<> {
public:

    DEFINE_LAYOUT_CONFIG_TYPES_OF(TRigidBody::LayoutConfigType);

    AddGyroTermVisitor(TRigidBody * body):
    m_rigidBody(body)
    {
        boost::apply_visitor(*this, m_rigidBody->m_geometry);
    }


    void operator()(boost::shared_ptr<const SphereGeometry<PREC> > & sphereGeom) {
        return;
    }

    void operator()(boost::shared_ptr<const BoxGeometry<PREC> > & box)  {
        addGyroTerm();
    }

    void operator()(boost::shared_ptr<const MeshGeometry<PREC> > & box) {
        addGyroTerm();
    }

    void operator()(boost::shared_ptr<const HalfspaceGeometry<PREC> > & halfspace) {
        addGyroTerm();
    }

    private:

    void addGyroTerm(){
        Vector3 K_omega_IK = m_rigidBody->m_pSolverData->m_uBuffer.m_Back.template tail<3>();
        m_rigidBody->m_h_term.template tail<3>() -= K_omega_IK.cross((m_rigidBody->m_K_Theta_S.asDiagonal() * K_omega_IK).eval());
    }

    TRigidBody * m_rigidBody;
};


#endif

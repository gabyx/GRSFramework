#ifndef InteriaTensorCalculations_hpp
#define InteriaTensorCalculations_hpp


#include <boost/variant.hpp>

#include "AssertionDebug.hpp"
#include "StaticAssert.hpp"

#include "SphereGeometry.hpp"
#include "BoxGeometry.hpp"
#include "HalfspaceGeometry.hpp"

namespace InertiaTensor{



    template<typename TRigidBody>
    class CalculateInertiaTensorVisitor : public boost::static_visitor<> {
    public:

        typedef typename TRigidBody::LayoutConfigType::PREC PREC;

        CalculateInertiaTensorVisitor(TRigidBody * body):
        m_rigidBody(body)
        {
            boost::apply_visitor(*this, m_rigidBody->m_geometry);
        }


        void operator()(boost::shared_ptr<const SphereGeometry<PREC> > & sphereGeom)  {
            m_rigidBody->m_K_Theta_S(0) = 2.0/5.0 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
            m_rigidBody->m_K_Theta_S(1) = 2.0/5.0 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
            m_rigidBody->m_K_Theta_S(2) = 2.0/5.0 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
        }

        void operator()(boost::shared_ptr<const BoxGeometry<PREC> > & box)  {
            m_rigidBody->m_K_Theta_S(0) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(2)*box->m_extent(2));
            m_rigidBody->m_K_Theta_S(1) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(0)*box->m_extent(0) + box->m_extent(2)*box->m_extent(2));
            m_rigidBody->m_K_Theta_S(2) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(0)*box->m_extent(0));
        }

        void operator()(boost::shared_ptr<const MeshGeometry<PREC> > & box)  {
            ASSERTMSG(false,"MeshGeometry InertiaCalculations: This has not been implemented yet!");
        }

        void operator()(boost::shared_ptr<const HalfspaceGeometry<PREC> > & halfspace)  {
            //This has not been implemented yet!
            ASSERTMSG(false,"HalfspaceGeometry InertiaCalculations: This has not been implemented yet!");
        }

        private:
        TRigidBody  * m_rigidBody;

    };

    template<typename TRigidBody>
    void calculateInertiaTensor( TRigidBody * rigidBody) {
        CalculateInertiaTensorVisitor<TRigidBody> vis(rigidBody);
    }


};



#endif

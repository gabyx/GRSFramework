#ifndef InteriaTensorCalculations_hpp
#define InteriaTensorCalculations_hpp


#include <boost/variant.hpp>

#include "TypeDefs.hpp"

#include "AssertionDebug.hpp"
#include "StaticAssert.hpp"

#include "SphereGeometry.hpp"
#include "BoxGeometry.hpp"
#include "HalfspaceGeometry.hpp"

namespace InertiaTensor{


    class CalculateInertiaTensorVisitor : public boost::static_visitor<> {
    public:

        DEFINE_RIGIDBODY_CONFIG_TYPES

        CalculateInertiaTensorVisitor(RigidBodyType * body):
        m_rigidBody(body)
        {
            boost::apply_visitor(*this, m_rigidBody->m_geometry);
        }


        void operator()(boost::shared_ptr<const SphereGeometry > & sphereGeom)  {
            m_rigidBody->m_K_Theta_S(0) = 2.0/5.0 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
            m_rigidBody->m_K_Theta_S(1) = 2.0/5.0 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
            m_rigidBody->m_K_Theta_S(2) = 2.0/5.0 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
        }

        void operator()(boost::shared_ptr<const BoxGeometry > & box)  {
            m_rigidBody->m_K_Theta_S(0) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(2)*box->m_extent(2));
            m_rigidBody->m_K_Theta_S(1) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(0)*box->m_extent(0) + box->m_extent(2)*box->m_extent(2));
            m_rigidBody->m_K_Theta_S(2) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(0)*box->m_extent(0));
        }

        void operator()(boost::shared_ptr<const MeshGeometry > & box)  {
            ASSERTMSG(false,"MeshGeometry InertiaCalculations: This has not been implemented yet!");
        }

        void operator()(boost::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ASSERTMSG(false,"HalfspaceGeometry InertiaCalculations: This has not been implemented yet!");
        }

        private:
        RigidBodyType  * m_rigidBody;

    };

    template<typename TRigidBody>
    void calculateInertiaTensor( TRigidBody * rigidBody) {
        CalculateInertiaTensorVisitor vis(rigidBody);
    }


};



#endif

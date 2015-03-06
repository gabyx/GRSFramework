#ifndef GRSF_Dynamics_General_InertiaTensorCalculations_hpp
#define GRSF_Dynamics_General_InertiaTensorCalculations_hpp


#include <boost/variant.hpp>

#include "GRSF/Common/TypeDefs.hpp"

#include "GRSF/Common/AssertionDebug.hpp"
#include "GRSF/Common/StaticAssert.hpp"

#include "GRSF/Dynamics/Collision/Geometry/SphereGeometry.hpp"
#include "GRSF/Dynamics/Collision/Geometry/BoxGeometry.hpp"
#include "GRSF/Dynamics/Collision/Geometry/HalfspaceGeometry.hpp"

namespace InertiaTensorComputations{


    template<typename TRigidBody>
    class CalculateInertiaTensorVisitor : public boost::static_visitor<> {
    public:

        using RigidBodyType = TRigidBody;

        CalculateInertiaTensorVisitor(RigidBodyType * body):
        m_rigidBody(body)
        {
            boost::apply_visitor(*this, m_rigidBody->m_geometry);
        }


        void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom)  {
            m_rigidBody->m_K_Theta_S(0) = 0.4 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
            m_rigidBody->m_K_Theta_S(1) = 0.4 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
            m_rigidBody->m_K_Theta_S(2) = 0.4 * m_rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
        }

        void operator()(std::shared_ptr<const BoxGeometry > & box)  {
            m_rigidBody->m_K_Theta_S(0) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(2)*box->m_extent(2));
            m_rigidBody->m_K_Theta_S(1) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(0)*box->m_extent(0) + box->m_extent(2)*box->m_extent(2));
            m_rigidBody->m_K_Theta_S(2) = 1.0/12.0 * m_rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(0)*box->m_extent(0));
        }

        void operator()(std::shared_ptr<const MeshGeometry > & box)  {
            ASSERTMSG(false,"MeshGeometry InertiaCalculations: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ASSERTMSG(false,"HalfspaceGeometry InertiaCalculations: This has not been implemented yet!");
        }

        private:
        RigidBodyType  * m_rigidBody;

    };
    template<typename TRigidBody>
    void calculateInertiaTensor( TRigidBody * rigidBody) {
        CalculateInertiaTensorVisitor<TRigidBody> vis(rigidBody);
    }


};


namespace MassComputations{



    template<typename TRigidBody>
    class CalculateMassVisitor : public boost::static_visitor<> {
    public:

        using RigidBodyType = TRigidBody;
        using PREC = typename RigidBodyType::PREC;

        CalculateMassVisitor(RigidBodyType * body, PREC density):
        m_rigidBody(body), m_density(density)
        {
            boost::apply_visitor(*this, m_rigidBody->m_geometry);
        }

        void operator()(std::shared_ptr<const SphereGeometry > & sphereGeom)  {
            PREC d = 2*sphereGeom->m_radius;
            m_rigidBody->m_mass = d*d*d/3.0*M_PI_2*m_density;
        }

        void operator()(std::shared_ptr<const BoxGeometry > & box)  {
            m_rigidBody->m_mass = box->m_extent(0)*box->m_extent(1)*box->m_extent(2) * m_density;
        }

        void operator()(std::shared_ptr<const MeshGeometry > & box)  {
            ASSERTMSG(false,"MeshGeometry InertiaCalculations: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ASSERTMSG(false,"HalfspaceGeometry InertiaCalculations: This has not been implemented yet!");
        }

        private:
        PREC m_density;
        RigidBodyType  * m_rigidBody;

    };

    template<typename TRigidBody,typename PREC>
    void calculateMass( TRigidBody * rigidBody, PREC density) {
        CalculateMassVisitor<TRigidBody> vis(rigidBody,density);
    }
};


#endif

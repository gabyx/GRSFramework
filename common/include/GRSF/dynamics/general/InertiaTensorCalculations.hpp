#ifndef GRSF_dynamics_general_InertiaTensorCalculations_hpp
#define GRSF_dynamics_general_InertiaTensorCalculations_hpp


#include <boost/variant.hpp>

#include "GRSF/common/TypeDefs.hpp"

#include "GRSF/common/AssertionDebug.hpp"
#include "GRSF/common/StaticAssert.hpp"

#include "GRSF/dynamics/collision/Geometries.hpp"


namespace InertiaTensorComputations{


    template<typename TRigidBody>
    class CalculateInertiaTensorVisitor : public boost::static_visitor<> {
    public:

        using RigidBodyType = TRigidBody;

        DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)

        CalculateInertiaTensorVisitor(RigidBodyType * body):
        m_rigidBody(body)
        {
            boost::apply_visitor(*this, m_rigidBody->m_geometry);
        }


        void operator()(SphereGeomPtrType & sphereGeom)  {
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
            ERRORMSG("MeshGeometry InertiaCalculations: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const CapsuleGeometry > & box)  {
            ERRORMSG("CapsuleGeometry InertiaCalculations: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ERRORMSG("HalfspaceGeometry InertiaCalculations: This has not been implemented yet!");
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
        DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)
        using PREC = typename RigidBodyType::PREC;

        CalculateMassVisitor(RigidBodyType * body, PREC density):
        m_rigidBody(body), m_density(density)
        {
            boost::apply_visitor(*this, m_rigidBody->m_geometry);
        }

        void operator()(SphereGeomPtrType & sphereGeom)  {
            PREC d = 2*sphereGeom->m_radius;
            m_rigidBody->m_mass = d*d*d/3.0*M_PI_2*m_density;
        }

        void operator()(std::shared_ptr<const BoxGeometry > & box)  {
            m_rigidBody->m_mass = box->m_extent(0)*box->m_extent(1)*box->m_extent(2) * m_density;
        }

        void operator()(std::shared_ptr<const MeshGeometry > & box)  {
            ERRORMSG("MeshGeometry CalculateMassVisitor: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const CapsuleGeometry > & box)  {
            ERRORMSG("CapsuleGeometry CalculateMassVisitor: This has not been implemented yet!");
        }

        void operator()(std::shared_ptr<const HalfspaceGeometry > & halfspace)  {
            //This has not been implemented yet!
            ERRORMSG("HalfspaceGeometry CalculateMassVisitor: This has not been implemented yet!");
        }

        private:
        RigidBodyType  * m_rigidBody;
        PREC m_density;


    };

    template<typename TRigidBody,typename PREC>
    void calculateMass( TRigidBody * rigidBody, PREC density) {
        CalculateMassVisitor<TRigidBody> vis(rigidBody,density);
    }
};


#endif

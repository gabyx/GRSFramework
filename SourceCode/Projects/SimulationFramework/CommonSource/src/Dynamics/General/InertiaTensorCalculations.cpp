
#include "InertiaTensorCalculations.hpp"

#include "AssertionDebug.hpp"

namespace InertiaTensor{


   void calculateInertiaTensor( const std::shared_ptr<MyRigidBodyType > & rigidBody)
   {
      using PREC = MyLayoutConfigType::PREC;

      if( std::shared_ptr<SphereGeometry<PREC> > sphereGeom = boost::get<std::shared_ptr<SphereGeometry<PREC> > >(rigidBody->m_geometry)){
         rigidBody->m_K_Theta_S(0) = 2.0/5.0 * rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
         rigidBody->m_K_Theta_S(1) = 2.0/5.0 * rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
         rigidBody->m_K_Theta_S(2) = 2.0/5.0 * rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
      }
      else if ( std::shared_ptr<BoxGeometry<PREC> > box = boost::get<std::shared_ptr<BoxGeometry<PREC> > >(rigidBody->m_geometry)){
         rigidBody->m_K_Theta_S(0) = 1.0/12.0 * rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(2)*box->m_extent(2));
         rigidBody->m_K_Theta_S(1) = 1.0/12.0 * rigidBody->m_mass * (box->m_extent(0)*box->m_extent(0) + box->m_extent(2)*box->m_extent(2));
         rigidBody->m_K_Theta_S(2) = 1.0/12.0 * rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(0)*box->m_extent(0));
      }
      else if(std::shared_ptr<HalfspaceGeometry<PREC> > halfspace = boost::get<std::shared_ptr<HalfspaceGeometry<PREC> > >(rigidBody->m_geometry)){
         ASSERTMSG(false,"You tried to compute an InertiaTensor for the type: HalfspaceGeometry which is not implemented yet!");
      }

   }

};

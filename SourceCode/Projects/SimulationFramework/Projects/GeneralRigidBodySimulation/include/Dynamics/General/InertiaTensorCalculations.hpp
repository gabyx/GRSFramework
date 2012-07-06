#ifndef InteriaTensorCalculations_hpp
#define InteriaTensorCalculations_hpp


#include <boost/variant.hpp>

#include "AssertionDebug.hpp"
#include "RigidBody.hpp"

namespace InertiaTensor{



   template<typename TLayoutConfig, typename TRigidBody>
   void calculateInertiaTensor( const boost::shared_ptr<TRigidBody > & rigidBody)
   {
      typedef typename TLayoutConfig::PREC PREC;

      if( boost::shared_ptr<SphereGeometry<PREC> > sphereGeom = boost::get<boost::shared_ptr<SphereGeometry<PREC> > >(rigidBody->m_geometry)){
         rigidBody->m_K_Theta_S(0) = 2.0/5.0 * rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
         rigidBody->m_K_Theta_S(1) = 2.0/5.0 * rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
         rigidBody->m_K_Theta_S(2) = 2.0/5.0 * rigidBody->m_mass * (sphereGeom->m_radius*sphereGeom->m_radius);
      }
      else if ( boost::shared_ptr<BoxGeometry<PREC> > box = boost::get<boost::shared_ptr<BoxGeometry<PREC> > >(rigidBody->m_geometry)){
         rigidBody->m_K_Theta_S(0) = 1.0/12.0 * rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(2)*box->m_extent(2));
         rigidBody->m_K_Theta_S(1) = 1.0/12.0 * rigidBody->m_mass * (box->m_extent(0)*box->m_extent(0) + box->m_extent(2)*box->m_extent(2));
         rigidBody->m_K_Theta_S(2) = 1.0/12.0 * rigidBody->m_mass * (box->m_extent(1)*box->m_extent(1) + box->m_extent(0)*box->m_extent(0));
      }
      else if(boost::shared_ptr<HalfspaceGeometry<PREC> > halfspace = boost::get<boost::shared_ptr<HalfspaceGeometry<PREC> > >(rigidBody->m_geometry)){
         ASSERTMSG(false,"You tried to compute an InertiaTensor for the type: HalfspaceGeometry which is not implemented yet!");
      }

   }








};


#endif

#ifndef ConstVariantTest_hpp
#define ConstVariantTest_hpp

#include <iostream>
#include <boost/variant.hpp>
#include "TypeDefs.hpp"

#include "SphereGeometry.hpp"
#include "BoxGeometry.hpp"
#include "HalfspaceGeometry.hpp"

#include "RigidBody.hpp"

/** @{ */
template<typename TDynamicsSystemConfig>
class Collider : public boost::static_visitor<> {
public:


    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF( TDynamicsSystemConfig );


    bool checkCollision(const RigidBodyType * pBody1, const RigidBodyType * pBody2){

        // Resets always pColData pointer, if there is a collision it is not NULL
        // delegate shall handle the proper storage where it should be (contact graph :-))

        m_pBody1 = const_cast<RigidBodyType *>(pBody1);
        m_pBody2 = const_cast<RigidBodyType *>(pBody2);

        boost::apply_visitor(*this, pBody1->m_geometry, pBody2->m_geometry);
        return true;
    }

    //For RigidBodies
    void operator()(   const boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom1,
                         const boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom2); ///< Calls Sphere/Sphere collision detection.

    template <typename Geom1, typename Geom2>
    void operator()(const boost::shared_ptr<Geom1> &g1, const boost::shared_ptr<Geom2> &g2);
    // =================================================================================


private:
     RigidBodyType* m_pBody1; ///< Shared pointer to the first RigidBodyBase class instance.
     RigidBodyType* m_pBody2; ///< Shared pointer to the second RigidBodyBase class instance.

    // ===================================================================================================
};
/** @} */

// Dispatch =======================================================================================
template<typename TDynamicsSystem>
void Collider<TDynamicsSystem>::operator()(  const boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom1 ,
                                               const boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom2) {
                                                   std::cout << " sphere/sphere collider:" << std::endl;
}

template<typename TDynamicsSystem>
template <typename Geom1, typename Geom2>
void Collider<TDynamicsSystem>::operator()(const boost::shared_ptr<Geom1> &g1, const boost::shared_ptr<Geom2> &g2) {
    std::cout << " Standart collider:" << std::endl;
}


void constVariantTest(){

    Collider<MyConfigDynamicsSystem> coll;

    MyRigidBody * b1 = new MyRigidBody();
    b1->m_geometry = boost::shared_ptr<SphereGeometry<double> >(new SphereGeometry<double>(2) );

    MyRigidBody * b2 = new MyRigidBody();
    b2->m_geometry = boost::shared_ptr<SphereGeometry<double> >(new SphereGeometry<double>(3) );

    coll.checkCollision(b1,b2);

    const boost::shared_ptr<const SphereGeometry<double> > ptr = boost::get<boost::shared_ptr<const SphereGeometry<double> > >(b1->m_geometry);

}


#endif

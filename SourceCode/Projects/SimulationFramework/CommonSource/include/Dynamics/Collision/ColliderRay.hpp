#ifndef Collider_hpp
#define Collider_hpp

#include "boost/variant.hpp"
#include <boost/shared_ptr.hpp>

#include "ConfigureFile.hpp"

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#if USE_OPCODE == 1
#include <Opcode.h>
#else
#if USE_OZCOLLIDE == 1
#define OZCOLLIDE_PCH
#include <ozcollide/ozcollide.h>
#endif
#endif

#include RigidBody_INCLUDE_FILE

#include "AABB.hpp"
#include "CollisionData.hpp"

#include "QuaternionHelpers.hpp"
#include "MatrixHelpers.hpp"
#include "MakeCoordinateSystem.hpp"
#include "CollisionFunctions.hpp"


/**
* @ingroup Collision
* @brief This is the ColliderRay class, this functor class handles the collision of different RigidBodies with a ray.
*/
/** @{ */

class ColliderRay : public boost::static_visitor<> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    /**
    * @brief The collider constructor which takes a reference to an existing collision set.
    */
    Collider():{
        m_bObjectsSwapped = false;
    }

    ~Collider(){

    }

    /**
    * @brief The initializer before this functor class should be used. This initializer is used to have two pointers to the RigidBodyBase classes
    * which are tested against each other.
    */

    void checkCollision(RigidBodyType * pBody1, RigidBodyType * pBody2) {


        // We know that we are not changing anything inside rigid body!
        // Otherwise all operators()(const boost::shared_ptr...)
        m_pBody1 = pBody1;
        m_pBody2 = pBody2;

        ASSERTMSG(m_pBody1 != m_pBody2, "Are you sure you want to checkCollision between the same objects?");
        m_bObjectsSwapped = false;
        m_bOverlapTest = false;
        m_bOverlap = false;
        boost::apply_visitor(*this, m_pBody1->m_geometry, m_pBody2->m_geometry);

    }

    bool checkOverlap(const RigidBodyType * pBody1, const AABB & aabb) {

        // We know that we are not changing anything inside rigid body!
        // Otherwise all operators()(const boost::shared_ptr...)
        m_pBody1 = pBody1;

        m_bOverlapTest = true;
        m_bOverlap = false;
        otherGeoms = &aabb;
        boost::apply_visitor(*this, m_pBody1->m_geometry, otherGeoms);
        return m_bOverlap;
    }

    bool intersectHalfray(const RigidBodyType * pBody1, const Vector3 & n, PREC & t) {

        // Inherit from ColliderRaycast if you need this functionality

    }


    /**
    * @name Dispatch operators
    * @brief Dispatch operators for the different boost::variant types defined in @ref RigidBodyBase.
    */
    /** @{ */


    //For RigidBodies
    inline void operator()(  const boost::shared_ptr<const SphereGeometry >  & sphereGeom1,
                             const boost::shared_ptr<const SphereGeometry >  & sphereGeom2); ///< Calls Sphere/Sphere collision detection.

    inline void operator()(  const boost::shared_ptr<const SphereGeometry >  & sphereGeom ,
                             const boost::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom); ///< Calls Sphere/Halfspace collision detection

    inline void operator()( const boost::shared_ptr<const BoxGeometry >  & box,
                            const boost::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom); ///< Calls Box/Halfsphere collision detection

    inline void operator()(  const boost::shared_ptr<const BoxGeometry >  & box1,
                             const boost::shared_ptr<const BoxGeometry >  & box2); ///< Calls Box/Box collision detection.

    inline void operator()(  const boost::shared_ptr<const SphereGeometry >  & sphere,
                             const boost::shared_ptr<const MeshGeometry >  & mesh); ///< Calls Mesh/Mesh collision detection.

    // For AABB's
    inline void operator()( const boost::shared_ptr<const SphereGeometry >  & sphereGeom1,
                            const AABB * aabb); ///< Calls Sphere/AABB collision detection.


    /**
    * @brief If no routine matched try to swap objects. If that fails too, an exception is thrown
    */
    template <typename Geom1, typename Geom2>
    inline void operator()(const boost::shared_ptr<const Geom1> &g1, const boost::shared_ptr<const Geom2> &g2);
    /** @} */

    /**
    * @brief If no routine matched for Body to AABB throw error
    */
    template <typename Geom1>
    inline void operator()(const  boost::shared_ptr<const Geom1> &g1, const AABB * aabb);
    /** @} */
    // =================================================================================


private:
    const RigidBodyType* m_pBody1; ///< Shared pointer to the first RigidBodyBase class instance.
    const RigidBodyType* m_pBody2; ///< Shared pointer to the second RigidBodyBase class instance.

    boost::variant<const AABB *> otherGeoms; ///< Used for other intersection tests



    bool m_bObjectsSwapped;                     ///< Boolean indicating if the bodies are swapped.
    bool m_bOverlapTest;                        ///< Boolean to decide if we only do overlap test or the whole collision output
    bool m_bOverlap;                            ///< Boolean which tells if the collision detection catched an overlap in the last call
    CollisionSetType * m_pColSet;               ///< List of found contacts for each query, gets cleard every time
    bool m_bDeleteColSet;                       ///< Boolean to decide if we own and should delete the m_pColSet pointer
    CollisionData * m_pColData;  ///< Temporary which is used always!
    /**
    * @brief The collision functions.
    * @{
    */
    //Collision Functions ===============================================================================
    // For RigidBodies
    inline void collide(const RigidBodyType * b1,
                        const boost::shared_ptr< const SphereGeometry >  & sphereGeom1,
                        const RigidBodyType * b2,
                        const boost::shared_ptr< const SphereGeometry >  & sphereGeom2); ///< Sphere/Sphere collision.

    inline void collide( const RigidBodyType * b1,
                         const boost::shared_ptr<const SphereGeometry >  & sphereGeom,
                         const RigidBodyType * b2,
                         const boost::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom); ///< Sphere/Halfspace collision.

    inline void collide(const RigidBodyType * a,
                        const boost::shared_ptr<const BoxGeometry >  & boxA,
                        const RigidBodyType * b,
                        const boost::shared_ptr<const BoxGeometry >  & boxB); ///< Box/Box collision.

    inline void collide(const RigidBodyType * box,
                        const boost::shared_ptr<const BoxGeometry >  & boxGeom,
                        const RigidBodyType * halfspace,
                        const boost::shared_ptr<const HalfspaceGeometry >  &halfspaceGeom); ///< Box/Halfspace collision.

    inline void collide(const RigidBodyType * sphere,
                        const boost::shared_ptr<const SphereGeometry >  & sphereGeom,
                        const RigidBodyType * mesh,
                        const boost::shared_ptr<const MeshGeometry >  & meshGeom); ///< Sphere/Mesh collision.

    // For AABB's
    inline void collide( const RigidBodyType * sphere,
                         const boost::shared_ptr<const SphereGeometry >  & sphereGeom1,
                         const AABB* aabb); ///< Sphere/AABB collision

    template <typename O1, typename O2>
    inline void collide(const RigidBodyType * b1,
                        const boost::shared_ptr<const O1> & o1,
                        const RigidBodyType * b2,
                        const boost::shared_ptr<const O2>  & o2); ///< Exception, to indicate that no collision function could be matched, because its not implemented.

    /** @} */
    // ===================================================================================================
};
/** @} */

// ==============================================================================================================================================================================
// IMPLEMENTATION ===============================================================================================================================================================


// Inline Dispatch =======================================================================================

void Collider::operator()(  const boost::shared_ptr<const SphereGeometry >  & sphereGeom1 ,
                            const boost::shared_ptr<const SphereGeometry >  & sphereGeom2) {
    collide(m_pBody1, sphereGeom1, m_pBody2, sphereGeom2);
}


void Collider::operator()(  const boost::shared_ptr<const SphereGeometry >  & sphereGeom ,
                            const boost::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom) {
    collide(m_pBody1, sphereGeom, m_pBody2, halfspaceGeom);
}


void Collider::operator()(  const boost::shared_ptr<const BoxGeometry >  & box1 ,
                            const boost::shared_ptr<const BoxGeometry >  & box2) {
    collide(m_pBody1, box1, m_pBody2, box2);
}


void Collider::operator()(  const boost::shared_ptr<const BoxGeometry >  & box ,
                            const boost::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom) {
    collide(m_pBody1, box, m_pBody2, halfspaceGeom);
}


void Collider::operator()(  const boost::shared_ptr<const SphereGeometry >  & sphere ,
                            const boost::shared_ptr<const MeshGeometry >  & mesh) {
    collide(m_pBody1, sphere, m_pBody2, mesh);
}



void Collider::operator()(  const boost::shared_ptr<const SphereGeometry >  & sphereGeom1 , const AABB * aabb) {
    collide(m_pBody1, sphereGeom1, aabb);
}


template <typename Geom1, typename Geom2>
void Collider::operator()(const boost::shared_ptr<const Geom1> &g1, const  boost::shared_ptr<const Geom2> &g2) {
    m_bObjectsSwapped = true;
    collide(m_pBody2, (const boost::shared_ptr<const Geom2> &)g2, m_pBody1, (const boost::shared_ptr<const Geom1> &)g1);
}


template <typename Geom1>
void Collider::operator()(const boost::shared_ptr<const Geom1> &g1, const AABB * aabb) {
    ERRORMSG("Collider:: collision detection for object-combination "<< typeid(Geom1).name()<<" and AABB not supported!");
}
// ==================================================================================================


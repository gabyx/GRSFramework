#ifndef GRSF_Dynamics_Collision_Collider_hpp
#define GRSF_Dynamics_Collision_Collider_hpp

#include "boost/variant.hpp"
#include <memory>

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#if USE_OPCODE == 1
#include <Opcode.h>
#else
#if USE_OZCOLLIDE == 1
#define OZCOLLIDE_PCH
#include <ozcollide/ozcollide.h>
#endif
#endif

#include RigidBody_INCLUDE_FILE

#include "GRSF/Dynamics/Collision/CollisionData.hpp"
#include "GRSF/Dynamics/General/QuaternionHelpers.hpp"
#include "GRSF/Dynamics/General/MatrixHelpers.hpp"
#include "GRSF/Dynamics/General/MakeCoordinateSystem.hpp"
#include "GRSF/Dynamics/Collision/CollisionFunctions.hpp"



/**
* @ingroup Collision
* @brief This is the ColliderAABB class, this functor class handles the collision of different RigidBodies with an AABB.
* It only does overlap test and not a full collision test!
*/
/** @{ */

class ColliderAABBBase {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)
protected:

    mutable const AABB3d * m_aabb = nullptr;

    mutable bool m_bOverlapTest = false;                        ///< Boolean to decide if we only do overlap test or the whole collision output
    mutable bool m_bOverlap = false;                            ///< Boolean which tells if the collision detection catched an overlap in the last call

    mutable const RigidBodyType* m_pBody = nullptr; ///< Shared pointer to the first RigidBodyBase class instance.

public:
    inline bool overlapSphere(const Vector3 & p, PREC radius, const AABB3d & aabb) const{
        // Intersection test by Thomas Larsson "On Faster Sphere-Box Overlap Testing"
        // Using arvos overlap test because larsons gives false positives!
        PREC d = 0;
        PREC e,c;
        PREC r = radius;
        for(int i=0; i<3; i++) {
            c = p(i);
            e = std::max(aabb.m_minPoint(i)-c, 0.0) + std::max( c -  aabb.m_maxPoint(i) ,0.0);
            d = d + e*e;
        }
        if(d > r*r) {
            return false;
        }
        return true;
    }
};

class ColliderAABB : protected ColliderAABBBase, public boost::static_visitor<> {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)

    ColliderAABB() {}

    bool checkOverlap(const RigidBodyType * pBody1, const AABB3d & aabb) const {

        // We know that we are not changing anything inside rigid body!
        // Otherwise all operators()(const boost::shared_ptr...)
        m_pBody = pBody1;

        //m_bOverlapTest = true;
        m_bOverlap = false;

        m_aabb = &aabb;
        boost::apply_visitor(*this, m_pBody->m_geometry);
        return m_bOverlap;
    }

    // Dispatch
    void operator()(const SphereGeomPtrType  & sphereGeom1) const{
        m_bOverlap = overlapSphere(m_pBody->m_r_S, sphereGeom1->m_radius, *m_aabb);
    }

    /**
    * @brief If no routine matched for Body to AABB throw error
    */
    template <typename Geom1>
    inline void operator()(const  std::shared_ptr<const Geom1> &g1) const {
        ERRORMSG("ColliderAABB:: collision detection for object-combination "<< typeid(Geom1).name()<<" and AABB not supported!");
    }


};


/** @} */

/**
* @ingroup Collision
* @brief This is the ColliderOOBB class, this functor class handles the collision of different RigidBodies with an OOBB.
* It only does overlap test and not a full collision test!
*/
/** @{ */

class ColliderOOBB : protected ColliderAABBBase, public boost::static_visitor<> {
public:
    /**
    * Here aabb is in coordinates of frame K!
    */
    bool checkOverlap(const RigidBodyType * pBody1, const AABB3d & aabb, const Matrix33 & A_IK) const {

        m_pBody = pBody1;
        //m_bOverlapTest = true;
        m_bOverlap = false;
        m_aabb = &aabb;
        m_A_IK = &A_IK;

        m_pBody->m_geometry.apply_visitor(*this);
        return m_bOverlap;
    }

    // Dispatch
    void operator()(const SphereGeomPtrType  & sphereGeom1) const {
        // Transform the point of the body into frame K
        Vector3 p = m_A_IK->transpose() * m_pBody->m_r_S;
        m_bOverlap = overlapSphere(p, sphereGeom1->m_radius, *m_aabb);
    }

    /**
    * @brief If no routine matched for Body to OOBB throw error
    */
    template <typename Geom1>
    inline void operator()(const  std::shared_ptr<const Geom1> &g1) const {
        ERRORMSG("ColliderAABB:: collision detection for object-combination "<< typeid(Geom1).name()<<" and AABB not supported!");
    }

private:
    mutable const Matrix33 * m_A_IK; ///< Transformation from frame K to frame I where the body coordinates are represented in
};


/**
* @ingroup Collision
* @brief This is the ColliderKdTree class, this functor class handles the collision of different RigidBodies with a KdTree.
* It only does overlap test and not a full collision test!
*/
/** @{ */

template<typename TreeType>
class ColliderKdTree{
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)

    using NodeType = typename TreeType::NodeType;

    ColliderKdTree(){}
    ColliderKdTree(const NodeType * nonAddedLeaf): m_nonAddedLeaf(nonAddedLeaf){}

    inline void setNonAddedLeaf( const NodeType * nonAddedLeaf ){
        m_nonAddedLeaf = nonAddedLeaf;
        ASSERTMSG(m_nonAddedLeaf, "nullptr")
    }

    inline bool checkOverlapNode( const RigidBodyType * pBody,
                                  const NodeType * node,
                                  const Matrix33 & A_IK) const
    {
        // early rejection if aabb/oobb is not overlapped
        return m_colliderOOBB.checkOverlap(pBody, node->aabb(), A_IK );
    }

    inline bool checkOverlapNode(const RigidBodyType * pBody,
                                 const NodeType * node) const
    {
        // early rejection if aabb/oobb is not overlapped
        return m_colliderAABB.checkOverlap(pBody, node->aabb());
    }

    /** Caller is reponsible to clear overlapLeafIndices
    *   \p startNode Any non-leaf (normal) node (ancestor of m_nonAddedNode) to start recursion,
    *      returns only startNode if startNode is a leaf (without the guarantee that the body overlaps)
    *   If the algorithm encounters that the node m_nonAddedNode is overlapped it is not added to overlapLeafIndices
    *   \return if node m_nonAddedNode is overlapped or not
    */
    template<typename ResultIdxSet>
    inline bool checkOverlap(ResultIdxSet & overlapLeafIndices,
                              const NodeType * startNode,
                              const RigidBodyType * pBody,
                              const Matrix33 & A_IK) const
    {

        // early rejection if aabb/oobb is not overlapped
        // this test is crucial, if we dont do this
        // the body might lie outside of startNode, but we will
        // still get overlapping leafs below startNode because we only check left/right of splitAxis
        if( ! checkOverlapNode(pBody,startNode,A_IK) ){
            return false;  // no overlap at this nodes aabb
        }

        // m_nodeStack.clear() not necessary since it is always empty after this call
        m_nodeStack.emplace_back(startNode); // body overlaps this nodes aabb!
        return checkOverlap_imp<false,true>(overlapLeafIndices,pBody,&A_IK);
    }

    template<typename ResultIdxSet>
    inline bool checkOverlap(ResultIdxSet & overlapLeafIndices,
                            const NodeType * startNode,
                            const RigidBodyType * pBody) const
    {

        if( ! checkOverlapNode(pBody,startNode) ){
            return false;
        }
        m_nodeStack.emplace_back(startNode);
        return checkOverlap_imp<true,true>(overlapLeafIndices,pBody);
    }


    template<bool aligned, bool enableNonAddedLeaf = true, typename ResultIdxSet>
    inline bool checkOverlap_imp(ResultIdxSet & overlapLeafIndices,
                          const RigidBodyType * pBody,
                          const Matrix33 * A_IK = nullptr
                          ) const{

        const NodeType * currNode;
        char res;
        bool ret = false; // if m_nonAddedNode is overlapped

        // Breath first traversal
        while(!m_nodeStack.empty()){

            currNode = m_nodeStack.front();
            ASSERTMSG(currNode, "currNode is nullptr")

            if(!currNode->isLeaf()){

                // check if body overlaps halfspace of split wall
                if(aligned){
                    // TODO ugly const_cast, but fast,
                    m_isLRAligned.m_body = const_cast<RigidBodyType *>(pBody);
                    m_isLRAligned.m_splitAxis = currNode->getSplitAxis();
                    m_isLRAligned.m_splitPos  = currNode->getSplitPosition();
                    res = pBody->m_geometry.apply_visitor(m_isLRAligned);
                }else{
                    m_isLR.m_body = const_cast<RigidBodyType *>(pBody);
                    m_isLR.m_splitAxis = currNode->getSplitAxis();
                    m_isLR.m_splitPos  = currNode->getSplitPosition();
                    m_isLR.m_A_IK = const_cast<Matrix33*>(A_IK);
                    res = pBody->m_geometry.apply_visitor(m_isLR);
                }

                switch(res){
                    case 0: // left node overlap
                        m_nodeStack.emplace_back(currNode->leftNode());
                        break;
                    case 1: // right node overlap
                        m_nodeStack.emplace_back(currNode->rightNode());
                        break;
                    case 2: // both overlap
                        m_nodeStack.emplace_back(currNode->leftNode());
                        m_nodeStack.emplace_back(currNode->rightNode());
                        break;
                    default :
                        ERRORMSG("Strange value returned from left/right ")
                };

            }else{
                // we are at a leaf where body is overlapping
                // add to list
                if(enableNonAddedLeaf){
                    if(currNode != m_nonAddedLeaf){
                        overlapLeafIndices.emplace(currNode->getIdx());
                    }else{
                        ret = true;
                    }
                }else{
                    overlapLeafIndices.emplace(currNode->getIdx());
                }
            }

            m_nodeStack.pop_front();
        }
        return ret;
    }

    /** Visitor to check if geometry overlaps axis halfspace either left/right or both
    *   x >= m_splitPos belongs to right halfspace!
    *   x < m_splitPos belongs to left halfspace!
    */
    template<bool aligned>
    struct LeftRightAxisHalfspace: public boost::static_visitor<char>{

        inline char operator()(const SphereGeomPtrType  & sphereGeom) const{

            // Transform the point of the body into frame K
            // (m_A_IK->transpose() * m_body->m_r_S)(splitAxis) ;
            PREC posAxis;
            if(!aligned){
                posAxis = m_A_IK->col(m_splitAxis).dot(m_body->m_r_S);
            }else{
                posAxis = m_body->m_r_S(m_splitAxis);
            }

            if(posAxis < m_splitPos){ // on left side
                if( (posAxis + sphereGeom->m_radius) >= m_splitPos){
                    return 2; // overlap both
                }
                return 0; // overlap only left
            }else{ // on right side
                if( (posAxis - sphereGeom->m_radius) < m_splitPos){
                    return 2; // overlap both
                }
                return 1; // overlap only right
            }
        }

        /**
        * @brief If no routine matched for Body to OOBB throw error
        */
        template <typename Geom>
        inline char operator()(const  std::shared_ptr<const Geom> &g) const{
            ERRORMSG("ColliderAxisHalfspace:: collision detection for object-combination "<< typeid(Geom).name()<<" and Axis Halfspace not supported!");
            return 0;
        }

        mutable Matrix33 * m_A_IK;
        mutable RigidBodyType * m_body;
        mutable typename NodeType::SplitAxisType m_splitAxis;
        mutable PREC m_splitPos;
    };


private:

    LeftRightAxisHalfspace<true>       m_isLRAligned;
    LeftRightAxisHalfspace<false>      m_isLR;

    ColliderAABB m_colliderAABB;
    ColliderOOBB m_colliderOOBB;

    mutable std::deque<const NodeType *> m_nodeStack;

    const NodeType * m_nonAddedLeaf = nullptr;
};


/** @} */

/**
* @ingroup Collision
* @brief This is the ColliderRay class, this functor class handles the collision of different RigidBodies with an AABB.
*/
/** @{ */

class ColliderRay : public boost::static_visitor<> {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)
    ColliderRay() {}

    /** Intersects body with a ray. If there is a intersection collData is filled and ray is not changed!
    *   @return a pair which indicates if a intersection is found and if found the new line parameter t!
    */
    std::pair<bool,PREC> intersect(const RigidBodyType * pBody1, Ray & ray, CollisionData & collData) {
        m_bIntersectionSimple = false;

        m_pBody = pBody1;
        m_ray = &ray;
        m_pCollData = &collData;

        ERRORMSG("ColliderRay:: not implemented")
    }

    /** intersects body with a ray. If a new intersection is found, the new line parameter is set in the reference ray! */
    bool intersectSimple(const RigidBodyType * pBody1, Ray & ray) {

        // parameter t in [0,infinity]

        m_pBody = pBody1;
        m_ray = &ray;

        m_bIntersectionSimple = true;
        m_bIntersection = false;

        boost::apply_visitor(*this, m_pBody->m_geometry);
        return m_bIntersection;
    }

    // Dispatch
    inline void operator()( const std::shared_ptr<const HalfspaceGeometry >  & halfspace) { ///< Calls Halfspace/Ray collision detection.
        intersect(halfspace.get());
    }
    /**
    * @brief If no routine matched for Body to AABB throw error
    */
    template <typename Geom1>
    inline void operator()(const  std::shared_ptr<const Geom1> &g1) {
        ERRORMSG("ColliderRay:: collision detection for object-combination "<< typeid(Geom1).name()<<" and Ray not supported!");
    }

private:

    Ray * m_ray;
    CollisionData * m_pCollData;

    bool m_bIntersectionSimple;
    bool m_bIntersection; ///< Boolean which tells if the intersection test gave a feasible result.

    const RigidBodyType* m_pBody; ///< Shared pointer to the first RigidBodyBase class instance.

    //Collision function
    inline void intersect( const HalfspaceGeometry * halfspace); ///< Halfspace/Ray intersection
};

void ColliderRay::intersect( const HalfspaceGeometry * halfspace) {

    PREC nDotRay = halfspace->m_normal.dot(m_ray->m_d);
    if ( nDotRay == 0.0) {
        m_bIntersection = false;
        return;
    }

    PREC t = halfspace->m_normal.dot(m_pBody->m_r_S - m_ray->m_p) / nDotRay;

    if( t <m_ray->m_mint || t > m_ray->m_maxt ) {
        m_bIntersection = false;
        return;
    }

    if( m_bIntersectionSimple ) {
        m_ray->m_maxt = t;
    } else {
        // Fill collision Data;
        ASSERTMSG(false,"fill collision data here")
    }

}

/** @} */


/**
* @ingroup Collision
* @brief This is the ColliderPoint class, this functor class handles the collision of different RigidBodies with a point.
*/
/** @{ */

class ColliderPoint : public boost::static_visitor<> {
public:
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    ColliderPoint() {}


    /** intersects body with a point and set to the closest point on body if it intersects!  */
    bool intersectAndProx(const RigidBodyType * pBody1,  Vector3 & p) {

        m_pBody = pBody1;
        m_p = &p;

        m_bIntersection = false;
        boost::apply_visitor(*this, m_pBody->m_geometry);
        return m_bIntersection;
    }

    // Dispatch
    inline void operator()( const std::shared_ptr<const HalfspaceGeometry >  & halfspace) { ///< Calls Halfspace/Point collision detection.
        Vector3 n = m_pBody->m_A_IK * halfspace->m_normal;
        m_bIntersection = CollisionFunctions::collideHalfspacePointAndProx(m_pBody->m_r_S, n, *m_p);
    }

    inline void operator()( const std::shared_ptr<const SphereGeometry >  & sphere) { ///< Calls Halfspace/Point collision detection.
        m_bIntersection =  CollisionFunctions::collideSpherePointAndProx(m_pBody->m_r_S, sphere->m_radius, *m_p);
    }

    inline void operator()( const std::shared_ptr<const CapsuleGeometry >  & capsule) { ///< Calls Capsule/Point collision detection.
        const CapsuleGeometry * caps = capsule.get();
        Vector3 n = m_pBody->m_A_IK * caps->m_normal;
        m_bIntersection =  CollisionFunctions::collideCapsulePointAndProx(m_pBody->m_r_S, n ,caps->m_length, caps->m_radius, *m_p);
    }

    inline void operator()( const std::shared_ptr<const MeshGeometry >  & mesh) {
        static bool show = true;
        if( show ){
            WARNINGMSG(false,"Mesh-Point collision detection has not been implemented")
            show=false;
        }
        m_bIntersection = false;
    }

    /**
    * @brief If no routine matched for Body to AABB throw error
    */
    template <typename Geom1>
    inline void operator()(const  std::shared_ptr<const Geom1> &g1) {
        ERRORMSG("ColliderPoint:: collision detection for object-combination "<< typeid(Geom1).name()<<" and point not supported!");
    }

private:

    Vector3 * m_p;

    bool m_bIntersection; ///< Boolean which tells if the intersection test gave a feasible result.

    const RigidBodyType* m_pBody; ///< Shared pointer to the first RigidBodyBase class instance.

};



/** @} */


/**
* @ingroup Collision
* @brief This is the ColliderBody class, this functor class handles the collision of different RigidBodies.
    It initializes two RigidBodyBase pointers and then the collider class is used as a functor with boost::apply_visitor(...)
    Which then matches the corresponding operator() which then further calls the corresponding collision routine!
*/
/** @{ */

template<typename TCollisionSet>
class ColliderBody : public boost::static_visitor<> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_GEOMETRY_PTR_TYPES(RigidBodyType)
    using CollisionSetType = TCollisionSet;

    /**
    * @brief The collider constructor which takes a reference to an existing collision set.
    */
    ColliderBody(CollisionSetType * pColSet): m_pColSet(pColSet) {
        m_bDeleteColSet = false;
        m_bObjectsSwapped = false;
    }

    /**
    * @brief The collider constructor which constructs internally maintains its own collision set.
    */
    ColliderBody() {
        m_pColSet =  new CollisionSetType();
        m_bDeleteColSet = true;
        m_bObjectsSwapped = false;
    }

    ~ColliderBody() {
        if(m_bDeleteColSet) {
            delete m_pColSet;
        }
    }

    /**
    * @brief The initializer before this functor class should be used. This initializer is used to have two pointers to the RigidBodyBase classes
    * which are tested against each other.
    * This function returns a bunch of contact points as a set of CollisionData instances!
    */

    void checkCollision(RigidBodyType * pBody1, RigidBodyType * pBody2) {
        m_pBody[0] = pBody1;
        m_pBody[1] = pBody2;

        ASSERTMSG(m_pBody[0] != m_pBody[1], "Are you sure you want to checkCollision between the same objects?");
        m_bObjectsSwapped = false;

        boost::apply_visitor(*this, m_pBody[0]->m_geometry, m_pBody[1]->m_geometry);
    }

    /**
    * @name Dispatch operators
    * @brief Dispatch operators for the different boost::variant types defined in @ref RigidBodyBase.
    */
    /** @{ */


    //For RigidBodies

    void operator()(  const SphereGeomPtrType  & sphereGeom1 ,
                      const SphereGeomPtrType  & sphereGeom2) {
        collide(sphereGeom1.get(), sphereGeom2.get());
    }
    void operator()(  const SphereGeomPtrType  & sphereGeom ,
                      const std::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom) {
        collide(sphereGeom.get(), halfspaceGeom.get());
    }
    void operator()(  const SphereGeomPtrType  & sphereGeom ,
                      const std::shared_ptr<const CapsuleGeometry >  & capsuleGeom) {
        collide(sphereGeom.get(), capsuleGeom.get());
    }
    void operator()(  const std::shared_ptr<const BoxGeometry >  & box1 ,
                      const std::shared_ptr<const BoxGeometry >  & box2) {
        collide(box1.get(), box2.get());
    }
    void operator()(  const std::shared_ptr<const BoxGeometry >  & box ,
                      const std::shared_ptr<const HalfspaceGeometry >  & halfspaceGeom) {
        collide(box.get(), halfspaceGeom.get());
    }
    void operator()(  const SphereGeomPtrType  & sphere ,
                      const std::shared_ptr<const MeshGeometry >  & mesh) {
        collide(sphere.get(), mesh.get());
    }

    /** If no routine matched try to swap objects. If that fails too, an exception is thrown*/
    template <typename Geom1, typename Geom2>
    void operator()(const std::shared_ptr<const Geom1> &g1, const  std::shared_ptr<const Geom2> &g2) {
        m_bObjectsSwapped = true;
        std::swap(m_pBody[0],m_pBody[1]);
        collide((const std::shared_ptr<const Geom2> &)g2, (const std::shared_ptr<const Geom1> &)g1);
    }
    /** @} */
    // =================================================================================


private:
    const RigidBodyType* m_pBody[2]; ///< Shared pointer to the first RigidBodyBase class instance.

    boost::variant<const AABB3d *> otherGeoms; ///< Used for other intersection tests



    bool m_bObjectsSwapped;                     ///< Boolean indicating if the bodies are swapped.
    CollisionSetType * m_pColSet;               ///< List of found contacts for each query, gets cleard every time
    bool m_bDeleteColSet;                       ///< Boolean to decide if we own and should delete the m_pColSet pointer
    CollisionData * m_pColData;  ///< Temporary which is used always!

    /**
    * @brief The collision functions. First geometry belongs to first body, second to second body!
    * @{
    */
    inline void collide(const SphereGeometry * sphereGeom1,
                        const SphereGeometry * sphereGeom2); ///< Sphere/Sphere collision.
    inline void collideSphereSphere(PREC r1, PREC r2, const Vector3 &c1, const Vector3 &c2);

    inline void collide(const SphereGeometry * sphereGeom,
                        const HalfspaceGeometry *  halfspaceGeom); ///< Sphere/Halfspace collision.

    inline void collide(const SphereGeometry *  sphereGeom,
                        const CapsuleGeometry * capsuleGeom); ///< Sphere/Capsule collision.

    inline void collide(const BoxGeometry  * boxA,
                        const BoxGeometry  * boxB); ///< Box/Box collision.

    inline void collide(const BoxGeometry  * boxGeom,
                        const HalfspaceGeometry *  halfspaceGeom); ///< Box/Halfspace collision.

    inline void collide(const SphereGeometry  * sphereGeom,
                        const MeshGeometry *  meshGeom); ///< Sphere/Mesh collision.


    /** Exception, to indicate that no collision function could be matched, because its not implemented. */
    template <typename O1, typename O2>
    inline void collide(const std::shared_ptr<const O1> & o1,
                        const std::shared_ptr<const O2> & o2) {
        ERRORMSG("ColliderBody:: collision detection for object-combination "<< typeid(O1).name()<<" and "<<typeid(O2).name()<<" not supported!");
    }

    /** @} */
    // ===================================================================================================
};
/** @} */

// ==============================================================================================================================================================================
// IMPLEMENTATION ===============================================================================================================================================================


// Collision Functions ==============================================================================
template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collide( const SphereGeometry * sphereGeom1,
                            const SphereGeometry * sphereGeom2) {
    // Do Collision for sphere to sphere
    collideSphereSphere(sphereGeom1->m_radius, sphereGeom2->m_radius,m_pBody[0]->m_r_S, m_pBody[1]->m_r_S);
}

/*
* Implementation of sphere-sphere collision, vectors of the centers c1 and c2 need to be in the same frame!
*/
template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collideSphereSphere(PREC r1, PREC r2, const Vector3 &c1, const Vector3 &c2) {

    Vector3 dist = c2 - c1; // I frame
    PREC dsqr = dist.squaredNorm();
    PREC rsqr = (r1 + r2);
    rsqr     *= rsqr;

    if(dsqr < rsqr) {

        //We have a collision
        m_pColSet->emplace_back();
        m_pColData = m_pColSet->back();

        //if the spheres are practically concentric just choose a random direction
        //to avoid division by zero
        if(dsqr < std::numeric_limits<PREC>::epsilon()) {
            dsqr = 0.0;
            m_pColData->m_cFrame.m_e_z = Vector3(0,0,1.0);
        }else{
            dsqr = sqrt(dsqr);
            m_pColData->m_cFrame.m_e_z = (1.0 / dsqr) * dist;
        }
        // dsqr is now the distance!
        // Coordinate system belongs to first body!
        CoordinateSystem::makeCoordinateSystem(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

        m_pColData->m_overlap = (r1 + r2) - dsqr;
        m_pColData->m_r_SC[0] =   m_pColData->m_cFrame.m_e_z * (r1 - m_pColData->m_overlap/2);
        m_pColData->m_r_SC[1] =  -m_pColData->m_cFrame.m_e_z * (r2 - m_pColData->m_overlap/2);


        // Set pointers
        m_pColData->m_pBody[0] = const_cast<RigidBodyType *>(m_pBody[0]);
        m_pColData->m_pBody[1] = const_cast<RigidBodyType *>(m_pBody[1]);

        // set Contact Tag
        m_pColData->m_contactTag.set(m_pBody[0]->m_id,0,0,m_pBody[1]->m_id,0,0);

    }
}


template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collide( const SphereGeometry * sphereGeom,
                            const HalfspaceGeometry * halfspaceGeom) {

    // Do Collision for sphere to halfspace
    Vector3 I_n_plane = m_pBody[1]->m_A_IK*halfspaceGeom->m_normal;

    double overlap = sphereGeom->m_radius - (m_pBody[0]->m_r_S - (  /*m_pBody[1]->m_A_IK * halfspaceGeom->m_pos  +*/  m_pBody[1]->m_r_S  )).dot( I_n_plane ) ;

    if(overlap >=0) {
        //We have a collision
        m_pColSet->emplace_back();
        m_pColData = m_pColSet->back();


        m_pColData->m_overlap = overlap;
        // Coordinate system belongs to first body!
        m_pColData->m_cFrame.m_e_z = - I_n_plane ;
        CoordinateSystem::makeCoordinateSystem(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

        m_pColData->m_r_SC[0] = (sphereGeom->m_radius - overlap/2) * m_pColData->m_cFrame.m_e_z ;
        m_pColData->m_r_SC[1] = ( m_pBody[0]->m_r_S + m_pColData->m_r_SC[0] ) - m_pBody[1]->m_r_S;

        // Set pointers
        m_pColData->m_pBody[0] = const_cast<RigidBodyType *>(m_pBody[0]);
        m_pColData->m_pBody[1] = const_cast<RigidBodyType *>(m_pBody[1]);

        // set Contact Tag
        m_pColData->m_contactTag.set(m_pBody[0]->m_id,0,0,m_pBody[1]->m_id,0,0);

    }

}

/*!\brief Contact generation between a Sphere and a Capsule.
 * In case of a contact between a sphere and a capsule, a single contact point is generated.
 * This contact point is calculated by first estimating a sphere within the capsule whose
 * center of mass is on the centerline of the capsule and closest to the center of mass of
 * the colliding sphere. After this, a sphere-sphere collision is performed between the
 * capsule representation and the colliding sphere.
 */
template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collide(const SphereGeometry *  sphereGeom,
                           const CapsuleGeometry * capsuleGeom) {

    const PREC halfL( 0.5*capsuleGeom->m_length );  // Half cylinder length


    // Calculating the component in the normal direction of the sphere in the frame K of the capsule
    // sphere->m_r_S - capsule->m_r_S  projecting onto the transformed normal in the I frame
    Vector3 I_normal(m_pBody[1]->m_A_IK*capsuleGeom->m_normal);
    PREC z = I_normal.dot(m_pBody[0]->m_r_S - m_pBody[1]->m_r_S);


    // Calculation the center of the sphere representing the capsule
    // limit the capsule representing sphere to the interval [-halfL, halfL]
    if( z > halfL ) {
        z = halfL;
    } else if( z < -halfL ) {
        z = -halfL;
    }

    I_normal = m_pBody[1]->m_r_S + z*I_normal;

    // Performing a sphere-sphere collision between the colliding sphere and the
    // capsule representation
    collideSphereSphere(sphereGeom->m_radius,capsuleGeom->m_radius, m_pBody[0]->m_r_S, I_normal);
}



template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collide( const BoxGeometry * boxA,
                            const BoxGeometry * boxB) {
    // Not implemented yet!
}


template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collide( const BoxGeometry * boxGeom,
                            const HalfspaceGeometry * halfspaceGeom) {


    // Check all 8 corners against the plane

    Vector3 I_n_plane = m_pBody[1]->m_A_IK*halfspaceGeom->m_normal;

    Vector3 r_SC1,r_SC2, temp1,temp2;
    temp1 = m_pBody[0]->m_A_IK*(boxGeom->m_center);
    temp2 = (m_pBody[0]->m_r_S) - (m_pBody[1]->m_r_S);


    for(int i=0; i<8; i++) {

        r_SC1 = temp1 + m_pBody[0]->m_A_IK*(boxGeom->getPoint(i));
        r_SC2 = r_SC1+temp2;

        double overlap = /*halfspaceGeom->m_normal.dot(halfspaceGeom->m_pos)*/ - ( r_SC2 ).dot( I_n_plane ) ;
        if(overlap >=0) {
            //We have a collision
            m_pColSet->emplace_back();
            m_pColData = m_pColSet->back();



            m_pColData->m_overlap = overlap;
            // Coordinate system belongs to first body!
            m_pColData->m_cFrame.m_e_z = - I_n_plane ;
            CoordinateSystem::makeCoordinateSystem(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

            m_pColData->m_r_SC[0] = r_SC1;
            m_pColData->m_r_SC[1] = r_SC2;

            // Set pointers
            m_pColData->m_pBody[0] = const_cast<RigidBodyType *>(m_pBody[0]);
            m_pColData->m_pBody[1] = const_cast<RigidBodyType *>(m_pBody[1]);

            // set Contact Tag
            m_pColData->m_contactTag.set(m_pBody[0]->m_id,0,0,m_pBody[1]->m_id,0,0);

        }
    }
}



template<typename TCollisionSet>
void ColliderBody<TCollisionSet>::collide(const SphereGeometry  * sphereGeom,
                           const MeshGeometry * meshGeom) {
    using namespace MatrixHelpers;

#if USE_OPCODE == 1
    // Collision detection with opcode!
//    Opcode::SphereCollider sphereCollider;
//    sphereCollider.SetFirstContact(false);
//    sphereCollider.SetTemporalCoherence(false);
//    sphereCollider.SetPrimitiveTests(true);
//
//    static Opcode::SphereCache sphereCache;
//
//    IceMaths::Sphere sphereTemp(IceMaths::Point(sphere->m_r_S(0),sphere->m_r_S(1),sphere->m_r_S(2)),sphereGeom->m_radius);
//
//    static MyMatrix<OPCODE_PRECISION>::Matrix44 H_IK; // worldMeshMatrix is H_IM= [A_IM | I_r_IM] if M is mesh in glocker Notation!
//    setHomogeneousTransform<PREC,MeshPREC>(mesh->m_A_IK, mesh->m_r_S,H_IK);
//
//    // Take care! Direct X Compliant stupid fucking matrices!!
//    IceMaths::Matrix4x4 * mat = (IceMaths::Matrix4x4 *)(H_IK.data());
//    if(!sphereCollider.Collide(sphereCache,sphereTemp,*(meshGeom->m_pOpcodeModel),nullptr, mat )) { //(const IceMaths::Matrix4x4 *)(H_IK.data())
//        ASSERTMSG(false,"Collision Sphere Mesh failed!");
//    }
//    /*
//     float max = 0;
//     for(int i = 0 ; i< meshGeom->m_pMeshData->m_Vertices.size();i++){
//        if(meshGeom->m_pMeshData->m_Vertices[i](2) > max){
//           max = meshGeom->m_pMeshData->m_Vertices[i](2);
//        }
//     }*/
//
//    //cout << sphere->m_r_S(2) - sphereGeom->m_radius << " of "<< max<< endl;
//    if(!sphereCollider.GetContactStatus()) {
//        return;
//    }
//
//    //cout << "Collision withe Mesh" <<endl;
//    unsigned int nTouchedPrims = sphereCollider.GetNbTouchedPrimitives();
//    const unsigned int * touchedPrims = sphereCollider.GetTouchedPrimitives();

#endif
#if USE_OZCOLLIDE == 1

    //Collide with PolyTree
//    using namespace ozcollide;
//    // Make a Sphere in K System
//    Vector3 K_r_MS = mesh->m_A_IK.transpose()*(sphere->m_r_S - mesh->m_r_S);
//    ozcollide::Sphere ozSphere( ozcollide::Vec3f(K_r_MS(0),K_r_MS(1),K_r_MS(2)), sphereGeom->m_radius);
//
//    // Collide
//    ozcollide::AABBTreePoly::SphereColResult ozResult;
//    meshGeom->m_pTreePoly->collideWithSphere(ozSphere,ozResult);
//    if(ozResult.polys_.size()>0){
//        std::cout << " Collision with " << ozResult.polys_.size() << "polygons..."<<std::endl;
//        std::cout << "---> Users:" << ozResult.users_[0] << "polygons..."<<std::endl;
//        const Vec3f* points = meshGeom->m_pTreePoly->getPointsList();
//        Vec3f point0 = points[ozResult.polys_[0]->getIndex(0)];
//        std::cout << "Point"<<point0.x << ","<<point0.y<<","<<point0.z<<","<<std::endl;
//    }
//    unsigned int nTouchedPrims ;
//    const unsigned int * touchedPrims;

#endif

//     Post process to get the contact set!
//    static Vector3 r_S1C1;
//    static std::vector< boost::tuple<double,Vector3,unsigned int,unsigned int> > temporarySet; //[ overlap, and normal from sphere center!, type, id] (see makeContactTag())
//    static boost::tuple<double,Vector3,unsigned int,unsigned int> tempColEntry;
//
//    temporarySet.reserve(3);
//    temporarySet.clear();
//
//    for(unsigned int i=0; i<nTouchedPrims; i++) {
//
//        r_S1C1 = CollisionFunctions::getClosestPoint_PointTriangle<TLayoutConfig>(sphere->m_r_S, *meshGeom->m_pMeshData, mesh->m_r_S, mesh->m_A_IK, touchedPrims[i],  tempColEntry.template get<2>(), tempColEntry. template get<3>() ) - sphere->m_r_S; // r_S1C1
//        tempColEntry.template get<0>() = sphereGeom->m_radius - r_S1C1.norm(); // Overlap
//        tempColEntry.template get<1>() = r_S1C1.normalized();
//        if(tempColEntry.template get<0>() >= 0) {
//            // We are completely sure we have a collision!
//            // Move into temporary collision set only if there is no similar contact which is close enough, tolerance = angle between normals!
//            for(unsigned int j=0; j<temporarySet.size(); j++) {
//                if( acos( temporarySet[j].template get<1>().dot( tempColEntry.template get<1>() )) < (5/180*M_PI)) {
//                    //cout << "Detected both times the same contact" <<endl;
//                    continue;
//                }
//            }
//            // Otherwise Push into set and continue!
//            temporarySet.push_back(tempColEntry);
//        }
//    }


#if USE_OWN_COLLISION_CODE == 1

    static typename CollisionFunctions::ClosestPointSet temporarySet; //[ overlap, and normal from sphere center!, type, id] (see makeContactTag())
    //Iterate over all faces and check if it overlaps sphere
    temporarySet.clear();
    temporarySet.reserve(3);
//    CollisionFunctions::getClosestPointsInRadius_PointMesh(   sphere->m_r_S,
//            sphereGeom->m_radius,
//            *meshGeom->m_pMeshData,
//            mesh->m_r_S,
//            mesh->m_A_IK,
//            temporarySet);

    //point with maximum overlap
    CollisionFunctions::getClosestPointInRadius_PointMesh(   m_pBody[0]->m_r_S,
            sphereGeom->m_radius,
            *meshGeom->m_pMeshData,
            m_pBody[1]->m_r_S,
            m_pBody[1]->m_A_IK,
            temporarySet);
#endif

//        if( temporarySet.size() == 2){
//            std::cout << "Contacts:" << temporarySet.size() <<std::endl;
//            std::cout << "Normal:" << temporarySet[0].template get<1>()<<std::endl;
//        }

    // Signal all remaining contacts int the temporary set!
    for(unsigned int j=0; j<temporarySet.size(); j++) {

        m_pColSet->emplace_back();
        m_pColData = m_pColSet->back();


        m_pColData->m_overlap = temporarySet[j].get<0>();
        // Coordinate system belongs to first body!
        m_pColData->m_cFrame.m_e_z = temporarySet[j].get<1>(); //needs not to be normalized
        CoordinateSystem::makeCoordinateSystem(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

        m_pColData->m_r_SC[0] = ( sphereGeom->m_radius - m_pColData->m_overlap/2) * m_pColData->m_cFrame.m_e_z ;
        m_pColData->m_r_SC[1] = ( m_pBody[0]->m_r_S + m_pColData->m_r_SC[0] ) - m_pBody[1]->m_r_S;

        // Set pointers
        m_pColData->m_pBody[0] = const_cast<RigidBodyType *>(m_pBody[0]);
        m_pColData->m_pBody[1] = const_cast<RigidBodyType *>(m_pBody[1]);

        // set Contact Tag
        m_pColData->m_contactTag.set(m_pBody[0]->m_id,0,0,m_pBody[1]->m_id,temporarySet[j].get<2>(),temporarySet[j].get<3>());

    }

}

#endif

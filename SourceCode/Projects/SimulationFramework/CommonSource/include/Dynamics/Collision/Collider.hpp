#ifndef Collider_hpp
#define Collider_hpp

#include "boost/variant.hpp"
#include <boost/shared_ptr.hpp>

#include "ConfigureFile.hpp"
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "CollisionData.hpp"

#if USE_OPCODE == 1
#include <Opcode.h>
#else
#if USE_OZCOLLIDE == 1
#define OZCOLLIDE_PCH
#include <ozcollide/ozcollide.h>
#endif
#endif


#include "AABB.hpp"
#include "SphereGeometry.hpp"
#include "BoxGeometry.hpp"
#include "HalfspaceGeometry.hpp"
#include "MeshGeometry.hpp"

#include "QuaternionHelpers.hpp"
#include "MatrixHelpers.hpp"
#include "MakeCoordinateSystem.hpp"
#include "CollisionFunctions.hpp"


/**
* @ingroup Collision
* @brief This is the Collider class, this functor class handles the collision of different RigidBodies.
    It initializes two RigidBodyBase pointers and then the collider class is used as a functor with boost::apply_visitor(...)
    Which then matches the corresponding operator() which then further calls the corresponding collision routine!
*/
/** @{ */
class Collider : public boost::static_visitor<> {
public:

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

    typedef std::vector<CollisionData<RigidBodyType>* > CollisionSetType;

    /**
    * @brief The collider constructor which takes a reference to an existing collision set.
    */
    Collider(CollisionSetType * pColSet);

    /**
    * @brief The collider constructor which constructs internally maintains its own collision set.
    */
    Collider();

    ~Collider();

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

    bool checkOverlap(RigidBodyType * pBody1, const AABB<LayoutConfigType> & aabb) {

        // We know that we are not changing anything inside rigid body!
        // Otherwise all operators()(const boost::shared_ptr...)
        m_pBody1 = pBody1;

        m_bOverlapTest = true;
        m_bOverlap = false;
        otherGeoms = &aabb;
        boost::apply_visitor(*this, m_pBody1->m_geometry, otherGeoms);
        return m_bOverlap;
    }


    /**
    * @name Dispatch operators
    * @brief Dispatch operators for the different boost::variant types defined in @ref RigidBodyBase.
    */
    /** @{ */


    //For RigidBodies
    inline void operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom1,
                             boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom2); ///< Calls Sphere/Sphere collision detection.

    inline void operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom ,
                             boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom); ///< Calls Sphere/Halfspace collision detection

    inline void operator()( boost::shared_ptr<const BoxGeometry<PREC> >  & box,
                            boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom); ///< Calls Box/Halfsphere collision detection

    inline void operator()(  boost::shared_ptr<const BoxGeometry<PREC> >  & box1,
                             boost::shared_ptr<const BoxGeometry<PREC> >  & box2); ///< Calls Box/Box collision detection.

    inline void operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphere,
                             boost::shared_ptr<const MeshGeometry<PREC> >  & mesh); ///< Calls Mesh/Mesh collision detection.

    // For AABB's
    inline void operator()( boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom1,
                            const AABB<LayoutConfigType> * aabb); ///< Calls Sphere/AABB collision detection.


    /**
    * @brief If no routine matched try to swap objects. If that fails too, an exception is thrown
    */
    template <typename Geom1, typename Geom2>
    inline void operator()(boost::shared_ptr<const Geom1> &g1, boost::shared_ptr<const Geom2> &g2);
    /** @} */

    /**
    * @brief If no routine matched for Body to AABB throw error
    */
    template <typename Geom1>
    inline void operator()(boost::shared_ptr<const Geom1> &g1, const AABB<LayoutConfigType> * aabb);
    /** @} */
    // =================================================================================


private:
    RigidBodyType* m_pBody1; ///< Shared pointer to the first RigidBodyBase class instance.
    RigidBodyType* m_pBody2; ///< Shared pointer to the second RigidBodyBase class instance.

    boost::variant<const AABB<LayoutConfigType> *> otherGeoms; ///< Used for other intersection tests

    bool m_bObjectsSwapped;                     ///< Boolean indicating if the bodies are swapped.
    bool m_bOverlapTest;                        ///< Boolean to decide if we only do overlap test or the whole collision output
    bool m_bOverlap;                            ///< Boolean which tells if the collision detection catched an overlap in the last call
    CollisionSetType * m_pColSet;               ///< List of found contacts for each query, gets cleard every time
    bool m_bDeleteColSet;                       ///< Boolean to decide if we own and should delete the m_pColSet pointer
    CollisionData<RigidBodyType> * m_pColData;  ///< Temporary which is used always!
    /**
    * @brief The collision functions.
    * @{
    */
    //Collision Functions ===============================================================================
    // For RigidBodies
    inline void collide(RigidBodyType * b1,
                        boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom1,
                        RigidBodyType * b2,
                        boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom2); ///< Sphere/Sphere collision.

    inline void collide( RigidBodyType * b1,
                         boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
                         RigidBodyType * b2,
                         boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom); ///< Sphere/Halfspace collision.

    inline void collide(RigidBodyType * a,
                        boost::shared_ptr<const BoxGeometry<PREC> >  & boxA,
                        RigidBodyType * b,
                        boost::shared_ptr<const BoxGeometry<PREC> >  & boxB); ///< Box/Box collision.

    inline void collide(RigidBodyType * box,
                        boost::shared_ptr<const BoxGeometry<PREC> >  & boxGeom,
                        RigidBodyType * halfspace,
                        boost::shared_ptr<const HalfspaceGeometry<PREC> >  &halfspaceGeom); ///< Box/Halfspace collision.

    inline void collide(RigidBodyType * sphere,
                        boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
                        RigidBodyType * mesh,
                        boost::shared_ptr<const MeshGeometry<PREC> >  & meshGeom); ///< Sphere/Mesh collision.

    // For AABB's
    inline void collide( RigidBodyType * sphere,
                         boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom1,
                         const AABB<LayoutConfigType>* aabb); ///< Sphere/AABB collision

    template <typename O1, typename O2>
    inline void collide(RigidBodyType * b1,
                        boost::shared_ptr<const O1> & o1,
                        RigidBodyType * b2,
                        boost::shared_ptr<const O2>  & o2); ///< Exception, to indicate that no collision function could be matched, because its not implemented.

    /** @} */
    // ===================================================================================================
};
/** @} */

// ==============================================================================================================================================================================

// IMPLEMENTATION ===============================================================================================================================================================


Collider::Collider(CollisionSetType * pColSet): m_pColSet(pColSet) {
    m_bDeleteColSet = false;
    m_bObjectsSwapped = false;
}


Collider::Collider(){
    m_pColSet =  new CollisionSetType();
    m_bDeleteColSet = true;
    m_bObjectsSwapped = false;
}


Collider::~Collider(){
    if(m_bDeleteColSet){
        // Clear all entries
        for( auto it = m_pColSet->begin(); it != m_pColSet->end(); it++) {
            delete (*it);
        }
        m_pColSet->clear();

        delete m_pColSet;
    }
}

// Dispatch =======================================================================================

void Collider::operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom1 ,
        boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom2) {
    collide(m_pBody1, sphereGeom1, m_pBody2, sphereGeom2);
}


void Collider::operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom ,
        boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom) {
    collide(m_pBody1, sphereGeom, m_pBody2, halfspaceGeom);
}


void Collider::operator()(  boost::shared_ptr<const BoxGeometry<PREC> >  & box1 ,
        boost::shared_ptr<const BoxGeometry<PREC> >  & box2) {
    collide(m_pBody1, box1, m_pBody2, box2);
}


void Collider::operator()(  boost::shared_ptr<const BoxGeometry<PREC> >  & box ,
        boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom) {
    collide(m_pBody1, box, m_pBody2, halfspaceGeom);
}


void Collider::operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphere ,
        boost::shared_ptr<const MeshGeometry<PREC> >  & mesh) {
    collide(m_pBody1, sphere, m_pBody2, mesh);
}



void Collider::operator()(  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom1 ,
        const AABB<LayoutConfigType> * aabb) {
    collide(m_pBody1, sphereGeom1, aabb);
}


template <typename Geom1, typename Geom2>
void Collider::operator()(boost::shared_ptr<const Geom1> &g1, boost::shared_ptr<const Geom2> &g2) {
    m_bObjectsSwapped = true;
    collide(m_pBody2, (boost::shared_ptr<const Geom2> &)g2, m_pBody1, (boost::shared_ptr<const Geom1> &)g1);
}


template <typename Geom1>
void Collider::operator()(boost::shared_ptr<const Geom1> &g1, const AABB<LayoutConfigType> * aabb) {
    ERRORMSG("Collider:: collision detection for object-combination "<< typeid(Geom1).name()<<" and AABB not supported!");
}
// ==================================================================================================



// Collision Functions ==============================================================================

void Collider::collide( RigidBodyType * b1,
        boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom1,
        RigidBodyType * b2,
        boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom2) {
    // Do Collision for sphere to sphere

    Vector3 dist = b2->m_r_S - b1->m_r_S; // I frame

    PREC dsqr = dist.dot(dist);
    PREC rsqr = (sphereGeom1->m_radius + sphereGeom2->m_radius);
    rsqr     *= rsqr;

    if(dsqr < rsqr) {

        //We have a collision
        m_pColData = new CollisionData<RigidBodyType>();
        m_pColSet->push_back( m_pColData );

        //if the spheres are practically concentric just choose a random direction
        //to avoid division by zero
        if(dsqr < std::numeric_limits<PREC>::epsilon()) {
            dsqr = 1.0;
            dist(0) = 0.0;
            dist(1) = 0.0;
            dist(2) = 1.0;

        }
        //we have a collision
        PREC d = sqrt(dsqr);

        m_pColData->m_cFrame.m_e_z = dist / d;
        // Coordinate system belongs to first body!
        makeCoordinateSystem<>(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

        m_pColData->m_overlap = (sphereGeom1->m_radius + sphereGeom2->m_radius) - d;
        m_pColData->m_r_S1C1 =   m_pColData->m_cFrame.m_e_z * (sphereGeom1->m_radius - m_pColData->m_overlap/2);
        m_pColData->m_r_S2C2 =  -m_pColData->m_cFrame.m_e_z * (sphereGeom2->m_radius - m_pColData->m_overlap/2);


        // Set pointers
        m_pColData->m_pBody1 = b1;
        m_pColData->m_pBody2 = b2;

        // set Contact Tag
        m_pColData->m_ContactTag = makeContactTag<RigidBodyType>(b1,0,0,b2,0,0);

    }
}


void Collider::collide( RigidBodyType * b1,
        boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
        RigidBodyType * b2,
        boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom) {



    // Do Collision for sphere to halfspace
    Vector3 I_n_plane = b2->m_A_IK*halfspaceGeom->m_normal;

    double overlap = sphereGeom->m_radius - (b1->m_r_S - (  b2->m_A_IK * halfspaceGeom->m_pos  +  b2->m_r_S  )).dot( I_n_plane ) ;

    if(overlap >=0) {
        //We have a collision
        m_pColData = new CollisionData<RigidBodyType>();
        m_pColSet->push_back( m_pColData );

        m_pColData->m_overlap = overlap;
        // Coordinate system belongs to first body!
        m_pColData->m_cFrame.m_e_z = - I_n_plane ;
        makeCoordinateSystem<>(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

        m_pColData->m_r_S1C1 = (sphereGeom->m_radius - overlap/2) * m_pColData->m_cFrame.m_e_z ;
        m_pColData->m_r_S2C2 = ( b1->m_r_S + m_pColData->m_r_S1C1 ) - b2->m_r_S;

        // Set pointers
        m_pColData->m_pBody1 = b1;
        m_pColData->m_pBody2 = b2;

        // set Contact Tag
        m_pColData->m_ContactTag = makeContactTag<RigidBodyType>(b1,0,0,b2,0,0);

    }

}


void Collider::collide(RigidBodyType * a,
                                        boost::shared_ptr<const BoxGeometry<PREC> >  & boxA,
                                        RigidBodyType * b,
                                        boost::shared_ptr<const BoxGeometry<PREC> >  & boxB) {
    // Not implemented yet!
}


void Collider::collide( RigidBodyType * box,
        boost::shared_ptr<const BoxGeometry<PREC> >  & boxGeom,
        RigidBodyType * halfspace,
        boost::shared_ptr<const HalfspaceGeometry<PREC> >  &halfspaceGeom) {


    // Check all 8 corners against the plane

    Vector3 I_n_plane = halfspace->m_A_IK*halfspaceGeom->m_normal;

    Vector3 r_SC1,r_SC2, temp1,temp2;
    temp1 = box->m_A_IK*(boxGeom->m_center);
    temp2 = (box->m_r_S) - (halfspace->m_r_S);

    double d = halfspaceGeom->m_normal.dot(halfspaceGeom->m_pos);
//            std::cout << "d:" << d << std::endl;
//            std::cout << "temp1:" << temp1  << std::endl;
    for(int i=0; i<8; i++) {
//                std::cout << boxGeom->getPoint(i) <<std::endl;
        r_SC1 = temp1 + box->m_A_IK*(boxGeom->getPoint(i));
        r_SC2 = r_SC1+temp2;

        double overlap = d - ( r_SC2 ).dot( I_n_plane ) ;
        if(overlap >=0) {
            //We have a collision
            m_pColData = new CollisionData<RigidBodyType>();
            m_pColSet->push_back( m_pColData );


            m_pColData->m_overlap = overlap;
            // Coordinate system belongs to first body!
            m_pColData->m_cFrame.m_e_z = - I_n_plane ;
            makeCoordinateSystem<>(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

            m_pColData->m_r_S1C1 = r_SC1;
            m_pColData->m_r_S2C2 = r_SC2;

            // Set pointers
            m_pColData->m_pBody1 = box;
            m_pColData->m_pBody2 = halfspace;

            // set Contact Tag
            m_pColData->m_ContactTag = makeContactTag<RigidBodyType>(box,0,0,halfspace,0,0);

        }
    }
}



void Collider::collide( RigidBodyType * sphere,
        boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
        RigidBodyType * mesh,
        boost::shared_ptr<const MeshGeometry<PREC> >  & meshGeom) {
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
//    if(!sphereCollider.Collide(sphereCache,sphereTemp,*(meshGeom->m_pOpcodeModel),NULL, mat )) { //(const IceMaths::Matrix4x4 *)(H_IK.data())
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

    static typename CollisionFunctions<LayoutConfigType >::ClosestPointSet temporarySet; //[ overlap, and normal from sphere center!, type, id] (see makeContactTag())
    //Iterate over all faces and check if it overlaps sphere
    temporarySet.clear();
    temporarySet.reserve(3);
//    CollisionFunctions<LayoutConfigType>::getClosestPointsInRadius_PointMesh(   sphere->m_r_S,
//            sphereGeom->m_radius,
//            *meshGeom->m_pMeshData,
//            mesh->m_r_S,
//            mesh->m_A_IK,
//            temporarySet);

    //point with maximum overlap
    CollisionFunctions<LayoutConfigType>::getClosestPointInRadius_PointMesh(   sphere->m_r_S,
            sphereGeom->m_radius,
            *meshGeom->m_pMeshData,
            mesh->m_r_S,
            mesh->m_A_IK,
            temporarySet);
#endif

//        if( temporarySet.size() == 2){
//            std::cout << "Contacts:" << temporarySet.size() <<std::endl;
//            std::cout << "Normal:" << temporarySet[0].template get<1>()<<std::endl;
//        }

    // Signal all remaining contacts int the temporary set!
    for(unsigned int j=0; j<temporarySet.size(); j++) {
        m_pColData = new CollisionData<RigidBodyType>();
        m_pColSet->push_back( m_pColData );

        m_pColData->m_overlap = temporarySet[j].get<0>();
        // Coordinate system belongs to first body!
        m_pColData->m_cFrame.m_e_z = temporarySet[j].get<1>(); //needs not to be normalized
        makeCoordinateSystem<>(m_pColData->m_cFrame.m_e_z,m_pColData->m_cFrame.m_e_x,m_pColData->m_cFrame.m_e_y);

        m_pColData->m_r_S1C1 = ( sphereGeom->m_radius - m_pColData->m_overlap/2) * m_pColData->m_cFrame.m_e_z ;
        m_pColData->m_r_S2C2 = ( sphere->m_r_S + m_pColData->m_r_S1C1 ) - mesh->m_r_S;

        // Set pointers
        m_pColData->m_pBody1 = sphere;
        m_pColData->m_pBody2 = mesh;

        // set Contact Tag
        m_pColData->m_ContactTag = makeContactTag<RigidBodyType>(sphere,0,0,mesh,temporarySet[j].get<2>(),temporarySet[j].get<3>());
    }

}


void Collider::collide(RigidBodyType * sphere,
                                        boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom1,
                                        const AABB<LayoutConfigType>* aabb) {
    //Intersection test by Thomas Larsson "On Faster Sphere-Box Overlap Testing"
    // Using arvos overlap test because larsons gives false positives!
    PREC d = 0;
    PREC e,c;
    PREC r = sphereGeom1->m_radius;
    m_bOverlap = false;
    for(int i=0; i<3; i++) {
        c = sphere->m_r_S(i);
        e = std::max(aabb->m_minPoint(i)-c, 0.0) + std::max( c -  aabb->m_maxPoint(i) ,0.0);
        d = d + e*e;
    }
    if(d > r*r) {
        return;
    }
    m_bOverlap = true;
    return;
}



template <typename O1, typename O2>
void Collider::collide(RigidBodyType * b1,
                                        boost::shared_ptr<const O1> & o1,
                                        RigidBodyType * b2,
                                        boost::shared_ptr<const O2>  & o2) {
    ERRORMSG("Collider:: collision detection for object-combination "<< typeid(O1).name()<<" and "<<typeid(O2).name()<<" not supported!");
}


//====================================================================================================


#endif

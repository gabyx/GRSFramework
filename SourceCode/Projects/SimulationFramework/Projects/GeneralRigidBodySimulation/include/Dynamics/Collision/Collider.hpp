#ifndef Collider_hpp
#define Collider_hpp


#include "boost/variant.hpp"
#include <boost/shared_ptr.hpp>
#include "boost/tuple/tuple.hpp"
#include <Eigen/Dense>

#include "ConfigureFile.hpp"
#include "AssertionDebug.hpp"

#include "TypeDefs.hpp"

#include "RigidBody.hpp"

#include "ContactGraph.hpp"
#include "CollisionData.hpp"

#include "QuaternionHelpers.hpp"
#include "MatrixHelpers.hpp"
#include "MakeCoordinateSystem.hpp"
#include "CollisionFunctions.hpp"

template<typename TLayoutConfig> class CollisionSolver;


/**
* @ingroup Collision
* @brief This is the Collider class, this functor class handles the collision of different RigidBodies.
	It initializes two RigidBody pointers and then the collider class is used as a functor with boost::apply_visitor(...)
	Which then matches the corresponding operator() which then further calls the corresponding collision routine!
*/
/** @{ */
template<typename TLayoutConfig, typename TCollisionSolver>
class Collider : public boost::static_visitor<>
{
public:
  DEFINE_LAYOUT_CONFIG_TYPES_OF(TLayoutConfig)

  /**
  * @brief The collider constructor which takes a reference to an existing collision set.
  */
  Collider();
  void init(TCollisionSolver * pCollisionSolver);
  /**
  * @brief The initializer before this functor class should be used. This initializer is used to have two pointers to the RigidBody classes
  * which are tested against each other.
  */
  void initializeBodies(boost::shared_ptr< RigidBody<TLayoutConfig> > &pBody1, boost::shared_ptr< RigidBody<TLayoutConfig> > &pBody2);

  /**
  * @name Dispatch operators
  * @brief Dispatch operators for the different boost::variant types defined in @ref RigidBody.
  */
  /** @{ */

  void operator()(  boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom1,
                    boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom2); ///< Calls Sphere/Sphere collision detection.

  void operator()(  boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom ,
                    boost::shared_ptr<HalfspaceGeometry<PREC> >  & halfspaceGeom); ///< Calls Sphere/Halfspace collision detection

  void operator()(  boost::shared_ptr<BoxGeometry<PREC> >  & box1,
                    boost::shared_ptr<BoxGeometry<PREC> >  & box2); ///< Calls Box/Box collision detection.

  void operator()(  boost::shared_ptr<SphereGeometry<PREC> >  & sphere,
                    boost::shared_ptr<MeshGeometry<PREC> >  & mesh); ///< Calls Mesh/Mesh collision detection.

  /**
  * @brief If no routine matched try to swap objects. If that fails too, an exception is thrown
  */
  template <typename Geom1, typename Geom2>
  void operator()(boost::shared_ptr<Geom1> &g1, boost::shared_ptr<Geom2> &g2);
  /** @} */
  // =================================================================================


private:
  boost::shared_ptr< RigidBody<TLayoutConfig> > m_pBody1; ///< Shared pointer to the first RigidBody class instance.
  boost::shared_ptr< RigidBody<TLayoutConfig> > m_pBody2; ///< Shared pointer to the second RigidBody class instance.
  bool m_bObjectsSwapped; ///< Boolean indicating if the bodies are swapped.

  TCollisionSolver * m_pCollisionSolver;
  /**
  * @brief The collision functions.
  * @{
  */
  //Collision Functions ===============================================================================
  void collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & b1,
                boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom1,
                boost::shared_ptr< RigidBody<TLayoutConfig> > & b2,
                boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom2); ///< Sphere/Sphere collision.

  void collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & b1,
                boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
                boost::shared_ptr<RigidBody<TLayoutConfig> > & b2,
                boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom); ///< Sphere/Halfspace collision.
  
  void collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & a,
                boost::shared_ptr<const BoxGeometry<PREC> >  & boxA,
                boost::shared_ptr<RigidBody<TLayoutConfig> > & b,
                boost::shared_ptr<const BoxGeometry<PREC> >  & boxB); ///< Box/Box collision.

  void collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & sphere,
                boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
                boost::shared_ptr<RigidBody<TLayoutConfig> > & mesh,
                boost::shared_ptr<const MeshGeometry<PREC> >  & meshGeom); ///< Box/Box collision.

  template <typename O1, typename O2>
  void collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & b1,
                boost::shared_ptr<const O1> & o1,
                boost::shared_ptr< RigidBody<TLayoutConfig> > & b2,
                boost::shared_ptr<const O2>  & o2); ///< Exception, to indicate that no collision function could be matched, because its not implemented.

  /** @} */
  // ===================================================================================================
};
/** @} */

// ==============================================================================================================================================================================

// IMPLEMENTATION ===============================================================================================================================================================

template<typename TLayoutConfig, typename TCollisionSolver>
  Collider<TLayoutConfig,TCollisionSolver>::Collider()                                 
  {
    m_bObjectsSwapped = false;
  }
template<typename TLayoutConfig, typename TCollisionSolver>
 void Collider<TLayoutConfig,TCollisionSolver>::init(TCollisionSolver * pCollisionSolver){
    m_pCollisionSolver = pCollisionSolver;
  }


template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::initializeBodies(boost::shared_ptr< RigidBody<TLayoutConfig> > &pBody1, boost::shared_ptr< RigidBody<TLayoutConfig> > &pBody2)
  {
    m_pBody1 = pBody1;
    m_pBody2 = pBody2;
    m_bObjectsSwapped = false;
  }

  // Dispatch =======================================================================================
template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::operator()(  boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom1 ,
                                      boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom2)
  {

      collide(m_pBody1, (boost::shared_ptr<const SphereGeometry<PREC> > &)sphereGeom1,
              m_pBody2, (boost::shared_ptr<const SphereGeometry<PREC> > &)sphereGeom2);
  }

template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::operator()(  boost::shared_ptr<SphereGeometry<PREC> >  & sphereGeom ,
                                      boost::shared_ptr<HalfspaceGeometry<PREC> >  & halfspaceGeom)
  {
      collide(m_pBody1, (boost::shared_ptr<const SphereGeometry<PREC> > &)sphereGeom,
              m_pBody2, (boost::shared_ptr<const HalfspaceGeometry<PREC> > &)halfspaceGeom);
  }

  template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::operator()(  boost::shared_ptr<BoxGeometry<PREC> >  & box1 ,
                                                              boost::shared_ptr<BoxGeometry<PREC> >  & box2)
  {
     ASSERTMSG(false,"No collision detection implemented for Box Box Collision!");
      collide(m_pBody1, (boost::shared_ptr<const BoxGeometry<PREC> > &)box1,
              m_pBody2, (boost::shared_ptr<const BoxGeometry<PREC> > &)box2);
  }

  template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::operator()(  boost::shared_ptr<SphereGeometry<PREC> >  & sphere ,
                                                              boost::shared_ptr<MeshGeometry<PREC> >  & mesh)
  {
      collide(m_pBody1, (boost::shared_ptr<const SphereGeometry<PREC> > &)sphere,
              m_pBody2, (boost::shared_ptr<const MeshGeometry<PREC> > &)mesh);
  }

template<typename TLayoutConfig, typename TCollisionSolver>
  template <typename Geom1, typename Geom2>
  void Collider<TLayoutConfig,TCollisionSolver>::operator()(boost::shared_ptr<Geom1> &g1, boost::shared_ptr<Geom2> &g2)
  {
    m_bObjectsSwapped = true;
    collide(m_pBody2, (boost::shared_ptr<const Geom2> &)g2, m_pBody1, (boost::shared_ptr<const Geom1> &)g1);
  }

// ==================================================================================================

 

// Collision Functions ==============================================================================
template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & b1,
                                  boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom1,
                                  boost::shared_ptr<RigidBody<TLayoutConfig> > & b2,
                                  boost::shared_ptr< const SphereGeometry<PREC> >  & sphereGeom2)
  {
    // Do Collision for sphere to sphere

    Vector3 dist = b2->m_r_S - b1->m_r_S; // I frame

    PREC dsqr = dist.dot(dist);
    PREC rsqr = (sphereGeom1->m_radius + sphereGeom2->m_radius);
    rsqr     *= rsqr;

    if(dsqr < rsqr)
    {

      //We have a collision
      CollisionData<TLayoutConfig>*  pColData = new CollisionData<TLayoutConfig>();

        //if the spheres are practically concentric just choose a random direction
        //to avoid division by zero
        if(dsqr < std::numeric_limits<PREC>::epsilon())
        {
            dsqr = 1.0;
            dist(0) = 0.0;
            dist(1) = 0.0;
            dist(2) = 1.0;
        }

        //we have a collision
        PREC d = sqrt(dsqr);

        pColData->m_e_z = dist / d;
        // Coordinate system belongs to first body!
        makeCoordinateSystem<>(pColData->m_e_z,pColData->m_e_x,pColData->m_e_y);

        pColData->m_overlap = (sphereGeom1->m_radius + sphereGeom2->m_radius) - d;
        pColData->m_r_S1C1 =   pColData->m_e_z * (sphereGeom1->m_radius - pColData->m_overlap/2);
        pColData->m_r_S2C2 =  -pColData->m_e_z * (sphereGeom2->m_radius - pColData->m_overlap/2);


        // Set pointers
        pColData->m_pBody1 = b1;
        pColData->m_pBody2 = b2;

        // set Contact Tag
        pColData->m_ContactTag = makeContactTag<TLayoutConfig>(b1.get(),0,0,b2.get(),0,0);


        m_pCollisionSolver->signalContactAdd(pColData);

    }
  }

template<typename TLayoutConfig, typename TCollisionSolver>
  void Collider<TLayoutConfig,TCollisionSolver>::collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & b1,
                                  boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
                                  boost::shared_ptr<RigidBody<TLayoutConfig> > & b2,
                                  boost::shared_ptr<const HalfspaceGeometry<PREC> >  & halfspaceGeom)
  {



    // Do Collision for sphere to halfspace
    Vector3 I_n_plane = b2->m_A_IK*halfspaceGeom->m_normal;

    double overlap = sphereGeom->m_radius - (b1->m_r_S - (  b2->m_A_IK * halfspaceGeom->m_pos  +  b2->m_r_S  )).dot( I_n_plane ) ;

    if(overlap >=0)
    {
      //We have a collision
      CollisionData<TLayoutConfig>*  pColData = new CollisionData<TLayoutConfig>();

      pColData->m_overlap = overlap;
      // Coordinate system belongs to first body!
      pColData->m_e_z = - I_n_plane ;
      makeCoordinateSystem<>(pColData->m_e_z,pColData->m_e_x,pColData->m_e_y);

      pColData->m_r_S1C1 = (sphereGeom->m_radius - overlap/2) * pColData->m_e_z ;
      pColData->m_r_S2C2 = ( b1->m_r_S + pColData->m_r_S1C1 ) - b2->m_r_S;

      // Set pointers
      pColData->m_pBody1 = b1;
      pColData->m_pBody2 = b2;

      // set Contact Tag
      pColData->m_ContactTag = makeContactTag<TLayoutConfig>(b1.get(),0,0,b2.get(),0,0);

      m_pCollisionSolver->signalContactAdd(pColData);
    }

  }

template<typename TLayoutConfig, typename TCollisionSolver>
void Collider<TLayoutConfig,TCollisionSolver>::collide(   boost::shared_ptr< RigidBody<TLayoutConfig> > & a,
                                                          boost::shared_ptr<const BoxGeometry<PREC> >  & boxA,
                                                          boost::shared_ptr<RigidBody<TLayoutConfig> > & b,
                                                          boost::shared_ptr<const BoxGeometry<PREC> >  & boxB)
{
   // Not implemented yet!
}


template<typename TLayoutConfig, typename TCollisionSolver>
void Collider<TLayoutConfig,TCollisionSolver>::collide(   boost::shared_ptr< RigidBody<TLayoutConfig> > & sphere,
                                                          boost::shared_ptr<const SphereGeometry<PREC> >  & sphereGeom,
                                                          boost::shared_ptr<RigidBody<TLayoutConfig> > & mesh,
                                                          boost::shared_ptr<const MeshGeometry<PREC> >  & meshGeom)
{
   using namespace MatrixHelpers;
   
   // Collision detection with opcode!
   Opcode::SphereCollider sphereCollider;
   sphereCollider.SetFirstContact(false);
   sphereCollider.SetTemporalCoherence(false);
	sphereCollider.SetPrimitiveTests(true);

   static Opcode::SphereCache sphereCache;

   IceMaths::Sphere sphereTemp(IceMaths::Point(sphere->m_r_S(0),sphere->m_r_S(1),sphere->m_r_S(2)),sphereGeom->m_radius);
   
   static MyMatrix<OPCODE_PRECISION>::Matrix44 H_IK; // worldMeshMatrix is H_IM= [A_IM | I_r_IM] if M is mesh in glocker Notation!
   setHomogeneousTransform<PREC,MeshPREC>(mesh->m_A_IK, mesh->m_r_S,H_IK);

   // Take care! Direct X Compliant stupid fucking matrices!!
   IceMaths::Matrix4x4 * mat = (IceMaths::Matrix4x4 *)(H_IK.data());
   if(!sphereCollider.Collide(sphereCache,sphereTemp,*(meshGeom->m_pOpcodeModel),NULL, mat )){ //(const IceMaths::Matrix4x4 *)(H_IK.data()) 
      ASSERTMSG(false,"Collision Sphere Mesh failed!");
   }
  /*
   float max = 0;
   for(int i = 0 ; i< meshGeom->m_pMeshData->m_Vertices.size();i++){
      if(meshGeom->m_pMeshData->m_Vertices[i](2) > max){
         max = meshGeom->m_pMeshData->m_Vertices[i](2);
      }
   }*/

   //cout << sphere->m_r_S(2) - sphereGeom->m_radius << " of "<< max<< endl;
   if(sphereCollider.GetContactStatus()){
      //cout << "Collision withe Mesh" <<endl;
      unsigned int nTouchedPrims = sphereCollider.GetNbTouchedPrimitives();
      const unsigned int * touchedPrims = sphereCollider.GetTouchedPrimitives();

      // Post process to get the contact set!
      static Vector3 r_S1C1;
      static std::vector< boost::tuple<double,Vector3,unsigned int,unsigned int> > temporarySet; //[ overlap, and normal from sphere center!, type, id] (see makeContactTag())
      static boost::tuple<double,Vector3,unsigned int,unsigned int> tempColEntry;

      temporarySet.reserve(3);
      temporarySet.clear();

      for(unsigned int i=0;i<nTouchedPrims;i++){
         
         r_S1C1 = CollisionFunctions::getClosestPoint_PointTriangle<TLayoutConfig>(sphere->m_r_S, *meshGeom->m_pMeshData, mesh->m_r_S, mesh->m_A_IK, touchedPrims[i],  tempColEntry.get<2>(), tempColEntry.get<3>() ) - sphere->m_r_S; // r_S1C1
         tempColEntry.get<0>() = sphereGeom->m_radius - r_S1C1.norm(); // Overlap
         tempColEntry.get<1>() = r_S1C1.normalized();
         if(tempColEntry.get<0>() >= 0){
            // We are completely sure we have a collision!
            // Move into temporary collision set only if there is no similar contact which is close enough, tolerance = angle between normals!
            for(int j=0;j<temporarySet.size();j++){
               if( acos( temporarySet[j].get<1>().dot( tempColEntry.get<1>() )) < (5/180*M_PI)){
                  //cout << "Detected both times the same contact" <<endl;
                  continue;
               }
            }
            // Otherwise Push into set and continue!
            temporarySet.push_back(tempColEntry);
         }
      }


      // Signal all remaining contacts int the temporary set!
      for(int j=0;j<temporarySet.size();j++){
            CollisionData<TLayoutConfig>*  pColData = new CollisionData<TLayoutConfig>();

            pColData->m_overlap = temporarySet[j].get<0>();
            // Coordinate system belongs to first body!
            pColData->m_e_z = temporarySet[j].get<1>();
            makeCoordinateSystem<>(pColData->m_e_z,pColData->m_e_x,pColData->m_e_y);

            pColData->m_r_S1C1 = ( sphereGeom->m_radius - pColData->m_overlap/2) * pColData->m_e_z ;
            pColData->m_r_S2C2 = ( sphere->m_r_S + pColData->m_r_S1C1 ) - mesh->m_r_S;

            // Set pointers
            pColData->m_pBody1 = sphere;
            pColData->m_pBody2 = mesh;

            // set Contact Tag
            pColData->m_ContactTag = makeContactTag<TLayoutConfig>(sphere.get(),0,0,mesh.get(),temporarySet[j].get<2>(),temporarySet[j].get<3>());

             m_pCollisionSolver->signalContactAdd(pColData);
      }

   }
  
}


template<typename TLayoutConfig, typename TCollisionSolver>
  template <typename O1, typename O2>
  void Collider<TLayoutConfig,TCollisionSolver>::collide( boost::shared_ptr< RigidBody<TLayoutConfig> > & b1,
                                boost::shared_ptr<const O1> & o1,
                                boost::shared_ptr< RigidBody<TLayoutConfig> > & b2,
                                boost::shared_ptr<const O2>  & o2)
  {
    ASSERTMSG(false,"Collider:: collision detection for object-combination not supported!");
  }


//====================================================================================================


#endif

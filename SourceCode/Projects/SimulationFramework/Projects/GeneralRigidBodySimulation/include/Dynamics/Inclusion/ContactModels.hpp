
#ifndef ContactModels_hpp
#define ContactModels_hpp

#include "ConvexSets.hpp"

#include <boost/any.hpp>

   /**
   * @brief Definitions of all contact models which can be used in the framework.
   * Because lack of time only  NormalAndCoulombFriction has been implemented in the solver.
   * The CollisionSolver determines what kind of Model the contact has and stores it in the CollisionData!
   */
   namespace ContactModels{

      /* 
      * @brief This is the friction model for a unilateral contact.
      * Abreviations : NContactModel
      */
      struct NormalContactModel{
         typedef NormalContactModel type;
         typedef ConvexSets::RPlus ConvexSet;
         static const int nDOFFriction = 0;
      };

    
      /* 
      * @brief This is the friction model for a unilateral contact with spatial coulomb friction.
      * Abreviations : NCFContactModel
      */
      struct NormalAndCoulombFrictionContactModel{
         typedef NormalAndCoulombFrictionContactModel type;
         typedef ConvexSets::RPlusAndDisk ConvexSet;
         static const int nDOFFriction = 2;
         static const int nFrictionParams = 1; // mu
      };
     
    

      /**
      * @brief This is the friction model for a unilateral contact with Coulombe-Contensou friction.
      * Abreviations : NCCFContactModel
      */
      struct NormalAndCoulombeContensouFrictionContactModel{
         typedef NormalAndCoulombeContensouFrictionContactModel type;
         typedef ConvexSets::RPlusAndContensouEllipsoid ConvexSet;
         static const int nDOFFriction = 3;
         static const int nFrictionParams = 2; // mu, r (?)
      };
     
    
      enum ContactModelEnum {
         NContactModel = 0,
         NCFContactModel = 1,
         NCCFContactModel = 2
      };


   };




#endif
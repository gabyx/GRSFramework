
#ifndef ContactModels_hpp
#define ContactModels_hpp

#include <boost/any.hpp>

#include "ConvexSets.hpp"

   /**
   * @brief Definitions of all contact models which can be used in the framework.
   * Because lack of time only  NormalAndCoulombFriction has been implemented in the solver.
   * The CollisionSolver determines what kind of Model the contact has and stores it in the CollisionData!
   */
   namespace ContactModels{

      /*
      * @brief This is the friction model for a unilateral contact.
      * Abreviations : N_ContactModel
      */
      struct NormalContactModel{
         typedef NormalContactModel type;
         typedef ConvexSets::RPlus ConvexSet;
         static const int nDOFFriction = 0;
      };


      /*
      * @brief This is the friction model for a unilateral contact with spatial coulomb friction.
      * Abreviations : NCF_ContactModel
      */
      struct NormalAndCoulombFrictionContactModel{
         typedef NormalAndCoulombFrictionContactModel type;
         typedef ConvexSets::RPlusAndDisk ConvexSet;
         static const int nDOFFriction = 2;
         static const int nFrictionParams = 1; // mu
      };



      /**
      * @brief This is the friction model for a unilateral contact with Coulombe-Contensou friction.
      * Abreviations : NCCF_ContactModel
      */
      struct NormalAndCoulombeContensouFrictionContactModel{
         typedef NormalAndCoulombeContensouFrictionContactModel type;
         typedef ConvexSets::RPlusAndContensouEllipsoid ConvexSet;
         static const int nDOFFriction = 3;
         static const int nFrictionParams = 2; // mu, r (?)
      };


      enum ContactModelEnum {
         N_ContactModel = 0,
         NCF_ContactModel = 1,
         NCCF_ContactModel = 2,
         B_ContactModel = 3, // Billateral Contact Model (not used yet)
      };


   };




#endif


#ifndef ContactModels_hpp
#define ContactModels_hpp


#include "ConvexSets.hpp"


   /**
   * @brief Definitions of all contact models which can be used in the framework.
   * Because lack of time only  NormalAndCoulombFriction has been implemented in the solver.
   * The CollisionSolver determines what kind of Model the contact has and stores it in the CollisionData!
   */
   namespace ContactModels{

      /*
      * @brief This is the friction model for a unilateral contact.
      * Abreviations : U_ContactModel
      */
      struct UnilateralContactModel{
         typedef ConvexSets::RPlus ConvexSet;
         static const int nDOFFriction = 0;
      };


      /*
      * @brief This is the friction model for a unilateral contact with spatial coulomb friction.
      * Abreviations : UCF_ContactModel
      */
      struct UnilateralAndCoulombFrictionContactModel{
         typedef ConvexSets::RPlusAndDisk ConvexSet;
         static const int nDOFFriction = 2;
         static const int nFrictionParams = 1; // mu
      };

       /*
      * @brief This is the friction model for a unilateral contact with spatial coulomb friction.
      * Abreviations : UCF_ContactModel
      */
      struct UnilateralAndCoulombFrictionDampedContactModel{
         typedef ConvexSets::RPlusAndDisk ConvexSet;
         static const int nDOFFriction = 2;
         static const int nFrictionParams = 1; // mu
         static const int nDampingParams = 2; // d1 normal and d2 tangential
      };




      /**
      * @brief This is the friction model for a unilateral contact with Coulombe-Contensou friction.
      * Abreviations : UCCF_ContactModel
      */
      struct UnilateralAndCoulombContensouFrictionContactModel{
         typedef ConvexSets::RPlusAndContensouEllipsoid ConvexSet;
         static const int nDOFFriction = 3;
         static const int nFrictionParams = 2; // mu, r (?)
      };


      enum class ContactModelEnum : short {
         U_ContactModel = 0,
         UCF_ContactModel = 1,
         UCFD_ContactModel = 2,
         UCCF_ContactModel = 3
      };


    constexpr unsigned int getLambdaDim(const ContactModelEnum & e){
        return (e == ContactModelEnum::U_ContactModel) ?  UnilateralContactModel::ConvexSet::Dimension :
            (
              (e == ContactModelEnum::UCF_ContactModel) ? UnilateralAndCoulombFrictionContactModel::ConvexSet::Dimension :

              (
                    (e == ContactModelEnum::UCFD_ContactModel) ? UnilateralAndCoulombFrictionDampedContactModel::ConvexSet::Dimension :
                    (
                        /*(e==ContactModelEnum::UCCF_ContactModel)?*/
                       +UnilateralAndCoulombContensouFrictionContactModel::ConvexSet::Dimension //(make an lvalue to rvalue conversion (all other static variables inherit this behaviour) with the unary+ operator, otherwise linking errors)

                    )
              )
            );
    }

   };




#endif

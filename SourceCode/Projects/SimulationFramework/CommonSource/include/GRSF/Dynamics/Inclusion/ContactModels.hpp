
#ifndef GRSF_Dynamics_Inclusion_ContactModels_hpp
#define GRSF_Dynamics_Inclusion_ContactModels_hpp


#include "GRSF/Dynamics/Inclusion/ConvexSets.hpp"


#include "GRSF/Common/EnumClassHelper.hpp"


#define CONTACTMODELTYPE( _EnumType_ ) ContactModels::ContactModel< EnumConversion::toIntegral( _EnumType_ ) >

   /**
   * @brief Definitions of all contact models which can be used in the framework.
   * Because lack of time only  NormalAndCoulombFriction has been implemented in the solver.
   * The CollisionSolver determines what kind of Model the contact has and stores it in the CollisionData!
   */
   namespace ContactModels{

    enum class Enum : short {
        U = 0,      /** Unilateral Contact */
        UCF = 1,    /** Unilateral Contact with Coulomb Friction*/
        UCFD = 2,   /** Unilateral Contact with Coulomb Friction and Damping */
        UCFDD = 3,  /** Unilateral Contact with Coulomb Friction and Damping (adjusts the slope) */
        UCCF = 4    /** Unilateral Contact with Coulomb Contensou Friction (not yet implemented) */
    };

    template<int T> struct ContactModel;

    /*
    * @brief UnilateralContactModel: This is the friction model for a unilateral contact.
    * Abreviations : U
    */
    template<>
    struct ContactModel< EnumConversion::template toIntegral(Enum::U)>{
        using ConvexSet = ConvexSets::RPlus;
        static const int nDOFFriction = 0;
        static const int epsNIdx = 0;
    };


    /*
    * @brief UnilateralAndCoulombFrictionContactModel: This is the friction model for a unilateral contact with spatial coulomb friction.
    * Abreviations : UCF
    */
    template<>
    struct ContactModel<EnumConversion::toIntegral(Enum::UCF)>{
        using ConvexSet = ConvexSets::RPlusAndDisk;
        static const int nDOFFriction = 2;
        static const int nFrictionParams = 1; // mu
        //Parameter offsets
        static const int epsNIdx = 0;
        static const int epsTIdx = 1;
        static const int muIdx = 2;
    };

    /*
    * @brief UnilateralAndCoulombFrictionDampedContactModel: This is the friction model for a unilateral contact with spatial coulomb friction
    * where for each law a damping is added serially.
    * Abreviations : UCFD
    */
    template<>
    struct ContactModel<EnumConversion::toIntegral(Enum::UCFD)>{
        using ConvexSet = ConvexSets::RPlusAndDisk;
        static const int nDOFFriction = 2;
        static const int nFrictionParams = 1; // mu
        static const int nDampingParams = 2; // d_N normal and d_T tangential

        //Parameter offsets
        static const int epsNIdx = 0;
        static const int epsTIdx = 1;
        static const int muIdx = 2;
        static const int d_NIdx = 3;
        static const int d_TIdx = 4;
    };

    /*
    * @brief UnilateralAndCoulombFrictionDampingDependendContactModel: This is the friction model for a unilateral contact with spatial coulomb friction
    * where for each law a damping is added serially (coulomb damping d_N = mu lambdaN / gamma_max is defined by a given maximal
    * relative velocity gamma_max (damping dependent on slip friction force lambda_N * mu)
    *
    * Abreviations : UCFDD
    */
    template<>
    struct ContactModel<EnumConversion::toIntegral(Enum::UCFDD)>{
        using ConvexSet = ConvexSets::RPlusAndDisk;
        static const int nDOFFriction = 2;
        static const int nFrictionParams = 1; // mu
        static const int nDampingParams = 4; // d_N normal and gamma_max tangential and epsilon and d_Tfix in the case where lambda_N <= epsilon

        //Parameter offsets
        static const int epsNIdx = 0;
        static const int epsTIdx = 1;
        static const int muIdx = 2;
        static const int d_NIdx = 3;
        static const int d_TfixIdx = 4;
        static const int gamma_maxIdx = 5;
        static const int epsIdx = 6;
    };




    /**
    * @brief This is the friction model for a unilateral contact with Coulomb-Contensou friction.
    * Abreviations : UCCF
    */
    template<>
    struct ContactModel<EnumConversion::toIntegral(Enum::UCCF)>{
        using ConvexSet = ConvexSets::RPlusAndContensouEllipsoid;
        static const int nDOFFriction = 3;
        static const int nFrictionParams = 2; // mu, r (?)
    };


    constexpr unsigned int getLambdaDim(const Enum & e){
        return (e == Enum::U) ?  CONTACTMODELTYPE(Enum::U)::ConvexSet::Dimension :
            (
              (e == Enum::UCF) ? CONTACTMODELTYPE(Enum::UCF)::ConvexSet::Dimension :

              (
                    (e == Enum::UCFD) ? CONTACTMODELTYPE(Enum::UCFD)::ConvexSet::Dimension :
                    (
                        (e == Enum::UCFDD) ? CONTACTMODELTYPE(Enum::UCFDD)::ConvexSet::Dimension :
                        (
                             /*(e==Enum::UCCF)?*/
                            +CONTACTMODELTYPE(Enum::UCCF)::ConvexSet::Dimension //(make an lvalue to rvalue conversion (all other static variables inherit this behaviour) with the unary+ operator, otherwise linking errors)

                        )

                    )
              )
            );
    }


};


#endif

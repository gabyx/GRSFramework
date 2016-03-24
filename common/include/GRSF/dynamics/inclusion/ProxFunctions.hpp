/*
*  GRSF/Dynamics/Inclusion/ProxFunctions.hpp
*
*  Created by Gabriel Nützi on 21.03.10.
*  Copyright 2010 ETH. All rights reserved.
*
*/

#ifndef GRSF_dynamics_inclusion_ProxFunctions_hpp
#define GRSF_dynamics_inclusion_ProxFunctions_hpp

#include <iostream>
#include <iomanip>

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/AssertionDebug.hpp"

#include "GRSF/dynamics/inclusion/ConvexSets.hpp"


#define INLINE_PROX 1

#if INLINE_PROX == 1
#define INLINE_PROX_KEYWORD inline
#else
#define INLINE_PROX_KEYWORD
#endif

/**
* @brief Namespace for the Prox Functions.
*/
namespace Prox {

/**
* @addtogroup Inclusion
* @{
*/

/**
* @defgroup ProxFunctions Prox Functions
* @brief Various proximal function onto different convex sets..
* @{
*/


// PROX SINGLE ====================================================================================================
/**
* @brief Base template for a single proximal functions on to several convex sets.
*/
template< typename Set >
struct ProxFunction {};

/**
* @brief Spezialisation for a  Prox onto \f$ C_1 = \mathcal{R}_{+} \f$ .
*/
template<>
struct ProxFunction<ConvexSets::RPlus> {

    /**
    * @brief Single in place Prox.
    * @param y Input/output which is proxed on to \f$ C_1\f$ .
    */
    template<typename PREC>
    static INLINE_PROX_KEYWORD void doProxSingle( PREC & y	) {
        using std::max;
        y = max(y,0.0);
    }
    /**
    * @brief Single Prox.
    * @param x Input vector
    * @param y Output which is proxed on to \f$ C_1\f$ .
    */
    template<typename PREC>
    static INLINE_PROX_KEYWORD void doProxSingle( const PREC & x, PREC & y	) {
        using std::max;
        y = max(x,0.0);
    }

    /**
    * @brief Multi in place Prox.
    * @param y Input/output which is proxed on to \f$ C_1\f$ .
    */
    template<typename Derived>
    static INLINE_PROX_KEYWORD void doProxMulti(const Eigen::MatrixBase<Derived> & y) {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        Eigen::MatrixBase<Derived> & y_ref =  const_cast<Eigen::MatrixBase<Derived> &>(y);

        for (unsigned int i=0; i<y_ref.rows(); i++) {
            using std::max;
            y_ref[i] = max(y_ref[i],0.0);
        }
    }

    /**
    * @brief Multi Prox.
    * @param x Input vector
    * @param y Output which is proxed on to \f$ C_1\f$ .
    */
    template<typename Derived, typename DerivedOther>
    static INLINE_PROX_KEYWORD void doProxMulti(const Eigen::MatrixBase<Derived> & x, const Eigen::MatrixBase<DerivedOther> & y) {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);

        Eigen::MatrixBase<DerivedOther> & y_ref =  const_cast<Eigen::MatrixBase<DerivedOther> &>(y);
        ASSERTMSG( x.rows() == y_ref.rows(), "Wrong dimension!");

        for (unsigned int i=0; i<y_ref.rows(); i++) {
            y_ref[i] = max(x[i],0.0);
        }
    }

};


/**
* @brief Spezialisation for a single Prox onto a scaled unit disk \f$ C_1 = \{ x | |x| < r \} \f$ .
*/
template<>
struct ProxFunction<ConvexSets::Disk> {

    /**
    * @brief Spezialisation for a single Prox onto a scaled unit disk \f$ C_1 \f$.
    * @param radius Scaling factor for the convex unit disk.
    * @param y Input/output vector which is proxed onto a scaled unit disk \f$ C_1 \f$.
    */
    template<typename PREC, typename Derived>
    static INLINE_PROX_KEYWORD void  doProxSingle(const PREC & radius,
                                                  const Eigen::MatrixBase<Derived> & y) {
        // Solve the set (disc with radius mu_P_N), one r is used for the prox!
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        ASSERTMSG(y.rows()%2==0,"wrong size");
        Eigen::MatrixBase<Derived> & y_ref =  const_cast<Eigen::MatrixBase<Derived> &>(y);

        PREC absvalue = y_ref.squaredNorm();
        if (absvalue > radius*radius) {
            y_ref *=   radius / std::sqrt(absvalue) ;
        }
    }

    /**
    * @brief Spezialisation for a single Prox onto a scaled unit disk  \f$ C_1 \f$.
    * @param radius Scaling factor for the convex unit disk.
    * @param x Input vector.
    * @param y Output which is proxed on to \f$ C_1 \f$.
    */
    template<typename PREC, typename Derived, typename DerivedOther>
    static INLINE_PROX_KEYWORD void  doProxSingle(const PREC & radius,
                                                  const Eigen::MatrixBase<Derived> & x,
                                                  const Eigen::MatrixBase<DerivedOther> & y) {
        // Solve the set (disc with radius mu_P_N), one r is used for the prox!
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
        ASSERTMSG(x.rows()%2==0,"wrong size");
        ASSERTMSG(y.rows()%2==0,"wrong size");
        Eigen::MatrixBase<DerivedOther> & y_ref =  const_cast<Eigen::MatrixBase<DerivedOther> &>(y);

        PREC absvalue;
        absvalue = x.squaredNorm();
        if (absvalue > radius*radius) {
            y_ref =  x * (radius/std::sqrt(absvalue));
        }
    }

    /**
    * @brief Spezialisation for a multi Prox onto a scaled unit disk \f$ C_1 \f$.
    * @param radius Scaling factor for the convex unit disk.
    * @param y Input/output vector which is proxed onto a scaled unit disk \f$ C_1 \f$.
    */
    template< typename Derived, typename DerivedOther>
    static INLINE_PROX_KEYWORD void doProxMulti(const Eigen::MatrixBase<Derived> & radius,
                                                const Eigen::MatrixBase<DerivedOther> & y) {
        using PREC = typename Derived::Scalar;
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
        ASSERTMSG(y.rows()%2==0,"wrong size");
        ASSERTMSG( (2) * radius.rows() == y.rows(), "Wrong dimension!");

        Eigen::MatrixBase<DerivedOther> & y_ref =  const_cast<Eigen::MatrixBase<DerivedOther> &>(y);
        //// Solve the set (disc with radius mu_P_N), one r is used for the prox!
        PREC absvalue;
        for (int i=0; i<radius.rows(); i++) {
            absvalue = (y_ref.segment<2>(2*i)).squaredNorm();
            if (absvalue > radius(i,0)*radius(i,0)) {
                y_ref.segment<2>(2*i) *=  (radius(i,0) / sqrt(absvalue));
            }
        }
    }

    /**
    * @brief Spezialisation for a multi Prox onto a scaled unit disk  \f$ C_1 \f$.
    * @param radius Scaling factor for the convex unit disk.
    * @param x Input vector.
    * @param y Output which is proxed on to \f$ C_1 \f$.
    */
    template< typename Derived, typename DerivedOther1, typename DerivedOther2>
    static INLINE_PROX_KEYWORD void doProxMulti(const Eigen::MatrixBase<Derived> & radius,
                                                const Eigen::MatrixBase<DerivedOther1> & x,
                                                const Eigen::MatrixBase<DerivedOther2> & y) {
        using PREC = typename Derived::Scalar;
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther1);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther2);
        ASSERTMSG(x.rows()%2==0,"wrong size");
        ASSERTMSG(y.rows()%2==0,"wrong size");
        ASSERTMSG( x.rows() == y.rows(), "Wrong dimension!");
        ASSERTMSG( (2) * radius.rows() == y.rows(), "Wrong dimension!");


        Eigen::MatrixBase<Derived> & y_ref =  const_cast<Eigen::MatrixBase<Derived> &>(y);


        // Solve the set (disc with radius mu_P_N), one r is used for the prox!
        PREC absvalue;
        for (int i=0; i<radius.rows(); i++) {
            absvalue = (x.segment<2>(2*i)).squaredNorm();
            if (absvalue > radius(i,0)*radius(i,0)) {
                y_ref.segment<2>(2*i) =  x.segment<2>(2*i) * (radius(i,0)/std::sqrt(absvalue));
            }
        }
    }

};

/**
* @brief Spezialisation for a single Prox onto  \f$ C_1 = \mathcal{R}_{+} \f$   and a scaled unit disk \f$ C_2 =\{ x | |x| < 1 \} \f$ . The scale factor \f$ r\f$  is the proxed value on to \f$ C_1 \f$ .
*/
template<>
struct ProxFunction<ConvexSets::RPlusAndDisk> {

    /** @brief Spezialisation for a single Prox onto  \f$ C_1 \f$  and \f$ C_2 \f$ .
    * @param scale_factor The scale_factor for scaling.
    * @param y Input/output vector, where the first value in y has been proxed onto  \f$ C_1 \f$  and the second 2 values onto the unit disk which is scaled by the first proxed value times scale_factor.
    */
    template<typename PREC, typename Derived>
    static INLINE_PROX_KEYWORD void doProxSingle(const PREC & scale_factor,
                                                 const Eigen::MatrixBase<Derived> & y) {

        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        ASSERTMSG(y.rows()==3,"wrong size");
        Eigen::MatrixBase<Derived> & y_ref =  const_cast<Eigen::MatrixBase<Derived> &>(y);

        //// Solve the set (disc with radius mu_P_N), one r is used for the prox!
        PREC absvalue;
        // Prox normal
        using std::max;
        y_ref(0) = max(y_ref(0),0.0);
        // Prox tangential
        absvalue = (y_ref.template segment<2>(1)).squaredNorm();
        if (absvalue > scale_factor*scale_factor*y_ref(0)*y_ref(0)) {
            y_ref.template segment<2>(1) *=    (scale_factor * y_ref(0)) / std::sqrt(absvalue);
        }
    }

    /**
    * @brief Spezialisation for a single Prox onto  \f$ C_1 \f$  and \f$ C_2 \f$ .
    * @param scale_factor The scale_factor for scaling.
    * @param x Input vector.
    * @param y Output vector, where the first value in y has been proxed onto  \f$ C_1 \f$
    * and the second 2 values onto the unit disk which is scaled by the first proxed value times scale_factor.
    */
    template<typename PREC, typename Derived, typename DerivedOther>
    static INLINE_PROX_KEYWORD void doProxSingle(const PREC & scale_factor,
                                                 const Eigen::MatrixBase<Derived> & x,
                                                 Eigen::MatrixBase<DerivedOther> & y) {
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
        ASSERTMSG(x.rows()==3,"wrong size");
        ASSERTMSG(y.rows()==3,"wrong size");
        Eigen::MatrixBase<Derived> & y_ref =  const_cast<Eigen::MatrixBase<Derived> &>(y);

        //// Solve the set (disc with radius mu_P_N), one r is used for the prox!
        PREC absvalue;
        // Prox normal
        using std::max;
        y_ref(0) = max(x(0),0.0);
        // Prox tangential
        absvalue = (x.template segment<2>(1)).squaredNorm();
        if (absvalue > scale_factor*scale_factor*y_ref(0)*y_ref(0)) {
            y_ref.template segment<2>(1) =  x.template segment<2>(1)  * (scale_factor*y_ref(0)/ std::sqrt(absvalue));
        }
    }

    /** @brief Spezialisation for a multi Prox onto  \f$ C_1 \f$  and \f$ C_2 \f$ .
    * @param scale_factor The scale_factor for scaling.
    * @param y Input/output vector, where the first value in y has been proxed onto  \f$ C_1 \f$
    * and the second 2 values onto the unit disk which is scaled by the first proxed value times scale_factor.
    */
    template< typename Derived, typename DerivedOther>
    static INLINE_PROX_KEYWORD void doProxMulti(const Eigen::MatrixBase<Derived> & scale_factor,
                                                const Eigen::MatrixBase<DerivedOther> & y) {
        using PREC = typename Derived::Scalar;
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
        ASSERTMSG(y.rows() % 3==0,"wrong size");
        Eigen::MatrixBase<DerivedOther> & y_ref =  const_cast<Eigen::MatrixBase<DerivedOther> &>(y);
        ASSERTMSG( (3) * scale_factor.rows() == y_ref.rows(), "Wrong dimension!");

        //// Solve the set (disc with radius mu_P_N), one r is used for the prox!
        PREC absvalue;
        for (int i=0; i<scale_factor.rows(); i++) {
            // Rplus
            using std::max;
            y_ref(3*i) = max(y_ref(3*i), 0.0);
            // Disk
            absvalue = (y_ref.template segment<2>(3*i+1)).squaredNorm();
            if (absvalue > scale_factor(i)*y_ref(3*i)*scale_factor(i)*y_ref(3*i)) {
                y_ref.template segment<2>(3*i+1) *=  scale_factor(i)*y_ref(3*i) / std::sqrt(absvalue);
            }
        }
    }

    /**
    * @brief Spezialisation for a multi Prox onto  \f$ C_1 \f$  and \f$ C_2 \f$ .
    * @param scale_factor The scale_factor for scaling.
    * @param x Input vector.
    * @param y Output vector, where the first value in y has been proxed onto  \f$ C_1 \f$
    *  and the second 2 values onto the unit disk which is scaled by the first proxed value time scale_factor.
    */
    template< typename Derived, typename DerivedOther1, typename DerivedOther2>
    static INLINE_PROX_KEYWORD void doProxMulti(const Eigen::MatrixBase<Derived> & scale_factor,
                                                const Eigen::MatrixBase<DerivedOther1> & x,
                                                const Eigen::MatrixBase<DerivedOther2> & y) {
        using PREC = typename Derived::Scalar;
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther1);
        EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther2);
        Eigen::MatrixBase<Derived> & y_ref =  const_cast<Eigen::MatrixBase<Derived> &>(y);
        ASSERTMSG(x.rows() % 3==0,"wrong size");
        ASSERTMSG(y.rows() % 3==0,"wrong size");
        ASSERTMSG( (3) * scale_factor.rows() == x.rows(), "Wrong dimension!");
        ASSERTMSG( (3) * scale_factor.rows() == y_ref.rows(), "Wrong dimension!");

        //// Solve the set (disc with radius mu_P_N), one r is used for the prox!
        PREC absvalue;
        for (int i=0; i<scale_factor.rows(); i++) {
            // Rplus
            using std::max;
            y_ref(3*i) = max(x(3*i),0.0);
            // Disk
            absvalue = (x.segment<2>(3*i+1)).squaredNorm();
            if (absvalue > scale_factor(i,0)*y_ref(3*i)*scale_factor(i,0)*y_ref(3*i)) {
                y_ref.segment<2>(3*i+1) =  x.segment<2>(3*i+1) * (scale_factor(i,0)*y_ref(3*i) / std::sqrt(absvalue)) ;
            }
        }
    }

};
/* @} */


/**
* @brief Spezialisation for a single Prox onto a Cone in \f$ \mathcal{R}^3 \f$ with center axis as the x-axis given as
* \f$ K = {(x_1,x_2,x_3) \in \mathcal{R}^3 \ | \ \sqrt{(x_2^2 + x_3^2)} \leq \mu x_1 \} \f$ .
* @param slopeFactor Friction coeff. \f$ \mu \f$ is the slope factor
*/
template<>
struct ProxFunction<ConvexSets::Cone3D> {

    template<typename PREC, typename Derived>
    static INLINE_PROX_KEYWORD void doProxSingle(const PREC & slopeFactor,
                                                 Eigen::MatrixBase<Derived> & y)
    {
            ASSERTMSG(y.rows() % 3==0,"wrong size");
            PREC normT = y.template tail<2>().norm();

            PREC testFricCone = slopeFactor*normT + y(0);

            if(normT - slopeFactor*y(0) <= 0.0){        // In Friction cone, do nothing!
                return;
            }else if(testFricCone <= 0.0) { // In polar cone to friction cone, set to zero!
                y.setZero();
                return;
            }

            // else project onto friction cone
            testFricCone /= (1+slopeFactor*slopeFactor);
            y(0) = testFricCone;
            y.template tail<2>() /= normT;
            y.template tail<2>() *= slopeFactor*testFricCone;

    }

};

/** @} */
};





// =====================================================================================================================

/**
* @brief Different numerical functions.
*/
namespace Numerics {

/**
* @addtogroup Inclusion
* @{
*/

// CANCEL FUNCTIONS ====================================================================================================
/**
* @defgroup CancelationFunctions Cancelation Functions
* @brief Cancelation function which are used to abort the iteration during the Prox iteration.
*/
/* @{ */
template<typename Derived>
INLINE_PROX_KEYWORD bool cancelCriteriaVector( const Eigen::MatrixBase<Derived>& P_N_old,
        const Eigen::MatrixBase<Derived>& P_N_new,
        const Eigen::MatrixBase<Derived>& P_T_old,
        const Eigen::MatrixBase<Derived>& P_T_new ,
        double AbsTol, double RelTol) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    using PREC = typename Derived::Scalar;

    using std::abs;
    using std::pow;
    using std::sqrt;

    PREC NormP =0;
    PREC RelNormP=0;
    for(int i=0; i<P_N_old.size(); i++) {
        NormP += P_N_old[i]*P_N_old[i];
        RelNormP += pow(P_N_new[i]-P_N_old[i],2);
    }

    for(int i=0; i<P_T_old.size(); i++) {
        NormP += P_T_old[i]*P_T_old[i];
        RelNormP += pow(P_T_new[i]-P_T_old[i],2);
    }

    NormP = sqrt(NormP);
    RelNormP = sqrt(RelNormP);

    if (RelNormP < NormP * RelTol + AbsTol) {
        return  true;
    }


    //LOGSLLEVEL3_CONTACT(m_pSolverLog, "Cancel Criterion :" << RelNormP << " < " << NormP * m_settings.m_RelTol + m_settings.m_AbsTol << std::endl;);


    return false;
}


template<typename Derived>
INLINE_PROX_KEYWORD bool cancelCriteriaValue(   const Eigen::MatrixBase<Derived>& P_N_old,
                                                const Eigen::MatrixBase<Derived>& P_N_new,
                                                const Eigen::MatrixBase<Derived>& P_T_old,
                                                const Eigen::MatrixBase<Derived>& P_T_new,
                                                double AbsTol, double RelTol) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);

    using std::abs;

    for(int i=0; i<P_N_old.size(); i++) {
        if ( abs(P_N_new[i]-P_N_old[i]) > abs(P_N_old[i]) * RelTol + AbsTol) {
            return  false;
        }
    }

    for(int i=0; i<P_T_old.size(); i++) {
        if ( abs(P_T_new[i]-P_T_old[i]) > abs(P_T_old[i]) * RelTol +AbsTol) {
            return  false;
        }
    }
    return true;
}

template<typename PREC, typename Derived, typename DerivedOther>
INLINE_PROX_KEYWORD bool cancelCriteriaValue(   const Eigen::MatrixBase<Derived>& P_old,
                                                const Eigen::MatrixBase<DerivedOther>& P_new,
                                                PREC AbsTol, PREC RelTol, PREC & residual) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    ASSERTMSG(P_old.rows()==P_new.rows(),"Vectors are not equal lenght!");

    using std::abs;
    residual = 0.0;
    PREC diff = 0.0;
    for(int i=0; i<P_old.size(); i++) {
//        residual = abs(P_new[i]-P_old[i]) - ( abs(P_old[i]) * RelTol + AbsTol);
//        if ( residual > 0.0) {
//            return  false;
//        }
        diff = abs(P_new[i]-P_old[i]) ;
        residual = std::max(diff,residual);
        if ( diff - ( abs(P_old[i]) * RelTol + AbsTol) > 0.0) {
            return  false;
        }
    }
    return true;
}

template<typename PREC, typename Derived, typename DerivedOther>
INLINE_PROX_KEYWORD bool cancelCriteriaValue(   const Eigen::MatrixBase<Derived>& P_old,
                                                const Eigen::MatrixBase<DerivedOther>& P_new,
                                                PREC AbsTol, PREC RelTol) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    ASSERTMSG(P_old.rows()==P_new.rows(),"Vectors are not equal lenght!");

    using std::abs;
    for(int i=0; i<P_old.size(); i++) {

        if ( abs(P_new[i]-P_old[i]) - ( abs(P_old[i]) * RelTol + AbsTol) > 0.0) {
            return  false;
        }
    }
    return true;
}


template<typename PREC, typename Derived, typename DerivedOther,typename DerivedOther2>
INLINE_PROX_KEYWORD bool cancelCriteriaMatrixNorm(  const Eigen::MatrixBase<Derived>& P_old,
                                                    const Eigen::MatrixBase<DerivedOther>& P_new,
                                                    const Eigen::MatrixBase<DerivedOther2> & NormMatrix_diag,
                                                    PREC AbsTol, PREC RelTol,
                                                    PREC & residual) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther2);
    ASSERTMSG(P_old.rows()==P_new.rows(),"Vectors are not equal lenght!");
    ASSERTMSG(NormMatrix_diag.rows()==P_old.rows(),"Vectors are not equal lenght!");

    using PRECM = typename Derived::Scalar;

    Eigen::Matrix<PRECM,Eigen::Dynamic, 1> diff = P_new - P_old; // Auslöschung!!! hm...

    using std::abs;
    using std::sqrt;
//    residual = sqrt(abs( (PREC)(diff.transpose()  * NormMatrix_diag.asDiagonal() * diff )))
//             - (sqrt(abs( (PREC)(P_old.transpose() * NormMatrix_diag.asDiagonal() * P_old))) * RelTol + AbsTol);
//    if ( residual > 0.0) {
//        return  false;
//    }

    residual = sqrt(abs( (PREC)(diff.transpose()  * NormMatrix_diag.asDiagonal() * diff )));
    if ( residual - (sqrt(abs( (PREC)(P_old.transpose() * NormMatrix_diag.asDiagonal() * P_old))) * RelTol + AbsTol) > 0.0) {
        return  false;
    }
    return true;
}

template<typename Derived, typename DerivedOther,typename DerivedOther2>
INLINE_PROX_KEYWORD bool cancelCriteriaMatrixNormSq(  const Eigen::MatrixBase<Derived>& P_old,
                                                    const Eigen::MatrixBase<DerivedOther>& P_new,
                                                    const Eigen::MatrixBase<DerivedOther2> & NormMatrix_diag,
                                                    double AbsTol, double RelTol,
                                                    double & residual) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther2);
    ASSERTMSG(P_old.rows()==P_new.rows(),"Vectors are not equal lenght!");
    ASSERTMSG(NormMatrix_diag.rows()==P_old.rows(),"Vectors are not equal lenght!");

    using PREC = typename Derived::Scalar;

    Eigen::Matrix<PREC,Eigen::Dynamic, 1> diff = P_new - P_old; // Auslöschung!!! hm...

    using std::abs;
    using std::sqrt;
//    residual = abs( (PREC)(diff.transpose()  * NormMatrix_diag.asDiagonal() * diff ))
//             - (abs( (PREC)(P_old.transpose() * NormMatrix_diag.asDiagonal() * P_old)) * RelTol + AbsTol);
//    if ( residual > 0.0) {
//        return  false;
//    }
    residual = abs( (PREC)(diff.transpose()  * NormMatrix_diag.asDiagonal() * diff ));
    if ( residual - (abs( (PREC)(P_old.transpose() * NormMatrix_diag.asDiagonal() * P_old)) * RelTol + AbsTol) > 0.0) {
        return  false;
    }
    return true;
}

template<typename Derived, typename DerivedOther,typename DerivedOther2>
INLINE_PROX_KEYWORD bool cancelCriteriaMatrixNormSq(  const Eigen::MatrixBase<Derived>& P_old,
                                                    const Eigen::MatrixBase<DerivedOther>& P_new,
                                                    const Eigen::MatrixBase<DerivedOther2> & NormMatrix_diag,
                                                    double AbsTol, double RelTol) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther2);
    ASSERTMSG(P_old.rows()==P_new.rows(),"Vectors are not equal lenght!");
    ASSERTMSG(NormMatrix_diag.rows()==P_old.rows(),"Vectors are not equal lenght!");

    using PREC = typename Derived::Scalar;

    Eigen::Matrix<PREC,Eigen::Dynamic, 1> diff = P_new - P_old; // Auslöschung!!! hm...

    using std::abs;
    using std::sqrt;
    if ( abs( (PREC)(diff.transpose()  * NormMatrix_diag.asDiagonal() * diff ))
             - (abs( (PREC)(P_old.transpose() * NormMatrix_diag.asDiagonal() * P_old)) * RelTol + AbsTol) > 0.0) {
        return  false;
    }
    return true;
}


template<typename PREC, typename Derived, typename DerivedOther,typename DerivedOther2, typename DerivedOther3>
INLINE_PROX_KEYWORD bool cancelCriteriaMatrixNorm(  const Eigen::MatrixBase<Derived>& u_old,
                                                    const Eigen::MatrixBase<DerivedOther2> & NormMatrix1_diag,
                                                    const Eigen::MatrixBase<DerivedOther>& P_old,
                                                    const Eigen::MatrixBase<DerivedOther>& P_new,
                                                    const Eigen::MatrixBase<DerivedOther3> & NormMatrix2,
                                                    PREC AbsTol, PREC RelTol) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther2);
    ASSERTMSG((P_old.rows()==P_new.rows()) && (NormMatrix1_diag.rows()==NormMatrix2.rows()) && (u_old.rows()==NormMatrix2.cols()),"Vectors are not equal lenght!");

    using PRECM = typename Derived::Scalar;
    //std::cout << " Convergence ENERGY" << std::endl;
    Eigen::Matrix<PRECM,Eigen::Dynamic, 1> diff = P_new - P_old; // Auslöschung!!! hm...

    using std::abs;
    using std::sqrt;
    if ( sqrt(abs( (PREC)(diff.transpose() * NormMatrix2 * diff ))) > sqrt(abs((PREC)(u_old.transpose() * NormMatrix1_diag.asDiagonal() * u_old))) * RelTol + AbsTol) {
        return  false;
    }
    return true;
}


template<typename PREC, typename Derived, typename DerivedOther>
INLINE_PROX_KEYWORD bool cancelCriteriaValue(   const Eigen::MatrixBase<Derived>& P_old,
                                                const Eigen::MatrixBase<DerivedOther>& P_new,
                                                PREC AbsTol, PREC RelTol,
                                                unsigned int & counter) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedOther);
    ASSERTMSG(P_old.rows()==P_new.rows(),"Vectors are not equal lenght!");

    using std::abs;

    for(int i=0; i<P_old.size(); i++) {
        if ( abs(P_new[i]-P_old[i]) > abs(P_old[i]) * RelTol + AbsTol) {
            return  false;
        }
    }
    counter++;
    return true;
}
/** @} */
/** @} */
}


#endif

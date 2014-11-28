/*
 *  QuternionMatrixTransform.h
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef QuternionHelpers_hpp
#define QuternionHelpers_hpp


#include "TypeDefs.hpp"

/**
* @ingroup Common
* @defgroup QuaternionHelpers Quaternion To Matrix Transformation
*/
/* @{ */

namespace QuaternionHelpers{

//Prototype
template<class Derived, class DerivedOther>
void setRotFromQuaternion(const Eigen::MatrixBase<Derived>& quat , const Eigen::MatrixBase<DerivedOther> &Rin);


/**
* @brief This returns a 3x3 rotation matrix from a 4x1 quaternion.
* @param quat The input quaternion.
* @return 3x3 rotation matrix.
*/
template<typename PREC>
typename MyMatrix<PREC>::Matrix33 getRotFromQuaternion(const typename MyMatrix<PREC>::Vector4 & quat) {
    //ASSERTMSG(quat.rows() == 4 && quat.cols()==1, "IN: "<< quat.rows()<<","<<quat.cols());

    typename MyMatrix<PREC>::Matrix33 A;
    //No check if quaternion is unit...(performance)
    setRotFromQuaternion(quat,A);
    return A;
}

/**
* @brief This sets the values of a 3x3 rotation matrix from a 4x1 quaternion.
* @param quat The input quaternion.
* @param A The output 3x3 rotation matrix.
*/
template<class Derived, class DerivedOther>
void setRotFromQuaternion(const Eigen::MatrixBase<Derived>& quat , const Eigen::MatrixBase<DerivedOther> &Rin) {

    Eigen::MatrixBase<DerivedOther> & R = const_cast< Eigen::MatrixBase<DerivedOther>& >(Rin);

    using PREC = typename Derived::Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,4);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedOther,3,3);
    //ASSERTMSG(quat.rows() == 4 && quat.cols()==1 &&  A.rows() == 3 && A.cols() == 3, "IN: "<< quat.rows()<<","<<quat.cols()<<"; OUT: "<<A.rows()<<","<<A.cols() );

    //No check if quaternion is unit...(performance)
    PREC fTx  = 2.0*quat(1);
    PREC fTy  = 2.0*quat(2);
    PREC fTz  = 2.0*quat(3);
    PREC fTwx = fTx*quat(0);
    PREC fTwy = fTy*quat(0);
    PREC fTwz = fTz*quat(0);
    PREC fTxx = fTx*quat(1);
    PREC fTxy = fTy*quat(1);
    PREC fTxz = fTz*quat(1);
    PREC fTyy = fTy*quat(2);
    PREC fTyz = fTz*quat(2);
    PREC fTzz = fTz*quat(3);

    R(0,0) = 1.0-(fTyy+fTzz);
    R(0,1) = fTxy-fTwz;
    R(0,2) = fTxz+fTwy;
    R(1,0) = fTxy+fTwz;
    R(1,1) = 1.0-(fTxx+fTzz);
    R(1,2) = fTyz-fTwx;
    R(2,0) = fTxz-fTwy;
    R(2,1) = fTyz+fTwx;
    R(2,2) = 1.0-(fTxx+fTyy);
}

//template<class Derived, class DerivedOther>
//void setQuaternion(const Eigen::MatrixBase<Derived>& nc_quat , const Eigen::MatrixBase<DerivedOther> & nc_n, const typename Derived::Scalar angleRadian) {
//    using PREC = typename Derived::Scalar;
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,4);
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedOther,3);
//
//    Eigen::MatrixBase<Derived>& quat = const_cast<Eigen::MatrixBase<Derived>& >( nc_quat);
//    Eigen::MatrixBase<DerivedOther>& n = const_cast<Eigen::MatrixBase<DerivedOther>& >( nc_n);
//
//    n.normalize();
//
//    quat(0) = cos(angleRadian/2);
//    quat.template tail<3>() = n * sin(angleRadian/2);
//
//}

template<class Derived, class DerivedOther>
void rotateVector(const Eigen::MatrixBase<Derived>& quat ,
                  Eigen::MatrixBase<DerivedOther> & v) {

    using PREC = typename Derived::Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,4);
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedOther,3);
    //ASSERTMSG(quat.rows() == 4 && quat.cols()==1 &&  A.rows() == 3 && A.cols() == 3, "IN: "<< quat.rows()<<","<<quat.cols()<<"; OUT: "<<A.rows()<<","<<A.cols() );

    //No check if quaternion is unit...(performance)
    PREC fTx  = 2.0*quat(1);
    PREC fTy  = 2.0*quat(2);
    PREC fTz  = 2.0*quat(3);
    PREC fTwx = fTx*quat(0);
    PREC fTwy = fTy*quat(0);
    PREC fTwz = fTz*quat(0);
    PREC fTxx = fTx*quat(1);
    PREC fTxy = fTy*quat(1);
    PREC fTxz = fTz*quat(1);
    PREC fTyy = fTy*quat(2);
    PREC fTyz = fTz*quat(2);
    PREC fTzz = fTz*quat(3); // 12 mult

    v(0) = (1.0-(fTyy+fTzz))*v(0)   +   (fTxy-fTwz)*v(1)        +    (fTxz+fTwy)*v(2); // 3mult + 6 add
    v(1) = (fTxy+fTwz)*v(0)          +   (1.0-(fTxx+fTzz))*v(1) +    (fTyz-fTwx)*v(2); // 3 mult + 6 add
    v(2) = (fTxz-fTwy)*v(0)          +   (fTyz+fTwx)*v(1)        +    (1.0-(fTxx+fTyy))*v(2); // 3 mult + 6 add
    // total = 21 mult , 18 add
}

template<class Derived>
void setQuaternionZero(Eigen::MatrixBase<Derived>& quat) {
    using PREC = typename Derived::Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,4);
    quat(0) = 1;
    quat(1) = 0;
    quat(2) = 0;
    quat(3) = 0;
}



template<class Derived>
Eigen::Matrix<typename Derived::Scalar,4,1> quatMult(const Eigen::MatrixBase<Derived>& quat1 , const Eigen::MatrixBase<Derived>& quat2) {
    using PREC = typename Derived::Scalar;
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,4);

    Eigen::Matrix< PREC ,4,1> ret;

    ret(0) =      quat1(0) * quat2(0) - quat1(1) * quat2(1) - quat1(2) * quat2(2) - quat1(3) * quat2(3);
    ret(1) =      quat1(0) * quat2(1) + quat1(1) * quat2(0) + quat1(2) * quat2(3) - quat1(3) * quat2(2);
    ret(2) =      quat1(0) * quat2(2) + quat1(2) * quat2(0) + quat1(3) * quat2(1) - quat1(1) * quat2(3);
    ret(3) =      quat1(0) * quat2(3) + quat1(3) * quat2(0) + quat1(1) * quat2(2) - quat1(2) * quat2(1);

    return ret;
}

}; // Quaternion

/* @} */
#endif

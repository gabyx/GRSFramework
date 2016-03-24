/*
 *  QuternionMatrixTransform.h
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef GRSF_dynamics_general_MatrixHelpers_hpp
#define GRSF_dynamics_general_MatrixHelpers_hpp

#include <boost/math/special_functions/fpclassify.hpp>

#include "GRSF/common/TypeDefs.hpp"

/**
* @ingroup Common
* @defgroup MatrixHelpers Matrix helper function
*/
/* @{ */


namespace MatrixHelpers{

   template<typename PREC, typename PREC2>
   void setHomogeneousTransform(const typename MyMatrix::Matrix33<PREC> &A_IK, const typename MyMatrix::Vector3<PREC> &I_t_IK, typename MyMatrix::Matrix44<PREC2> &H_IK){
      // Sets the matrix H_IK, ==> I_r_P = H_IK * K_r_p;
      H_IK.setIdentity();
      H_IK.block<3,3>(0,0) = A_IK.template cast<PREC2>();
      H_IK.block<3,1>(0,3) = I_t_IK.template cast<PREC2>();
   }

    template<typename Derived>
    bool isfinite( const Eigen::MatrixBase<Derived> & mat){
       EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
          for(int i=0;i<mat.cols();i++){
             if(!(boost::math::isfinite)(mat(i))){
               return false;
             }
          }
       return true;
    }

    /**
    * Set a 3x3 Matrix from 6 values, in = [a_00,a_01,a_02,a_11,a_12,a_22]
    */
    template<typename Derived, typename Derived2>
    void setSymMatrix(Eigen::MatrixBase<Derived>  &m , const Eigen::MatrixBase<Derived2> & in){
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived,3,3);
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived2,6);
        m(0,0)=in(0);
        m(0,1)=in(1); m(1,0)=in(1);
        m(0,2)=in(2); m(2,0)=in(2);
        m(1,1)=in(3);
        m(1,2)=in(4); m(2,1)=in(4);
        m(2,2)=in(5);
    }



};


/* @} */
#endif

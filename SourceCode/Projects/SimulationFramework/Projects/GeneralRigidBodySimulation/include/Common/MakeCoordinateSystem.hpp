#ifndef MakeCoordinateSystem_hpp
#define MakeCoordinateSystem_hpp

#include <cmath>

#include "TypeDefs.hpp"

/**
* @ingroup Common
* @defgroup MakeCoordinateSystem Make Coordinate System
* @brief This is a helper function to make a orthogonal normed right-hand coordinate system from an input vector.
*/
/* @{ */
/**
* @brief This function makes an orthogonal normed right-hand coordinate system. If the z-axis is the input, then v1 is the x-axis and v2 the y-axis.
* @param v1 The input 3x1 vector.
* @param v2 The first orthogonal output 3x1 vector.
* @param v2 The second orthogonal output 3x1 vector.
*/
template<typename Derived, typename DerivedOther1, typename DerivedOther2>
inline void makeCoordinateSystem(      Eigen::MatrixBase<Derived> & v1, 
                                       Eigen::MatrixBase<DerivedOther1> &v2, 
                                       Eigen::MatrixBase<DerivedOther2> &v3)
{
   EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3);
   EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedOther1,3);
   EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(DerivedOther2,3);
   /*ASSERTMSG(  v1.rows() == 3 && v1.cols()==1 &&  
               v2.rows() == 3 && v2.cols() == 1 &&
               v3.rows() == 3 && v3.cols() == 1 
               , "IN: "<< v1.rows()<<","<<v1.cols()<<" OUT: "<< v2.rows()<<","<<v2.cols()<<"/"<<v3.rows()<<","<<v3.cols() );*/

   typedef typename Derived::Scalar PREC;

   using std::abs;
   using std::sqrt;

   v1.normalize();

	if(abs(v1(0)) > abs(v1(2))){
    PREC invLen = 1.0 / sqrt(v1(0)*v1(0) + v1(2) * v1(2));
		v2 = typename MyMatrix<double>::Vector3(-v1(2) * invLen, 0, v1(0) *invLen);
	}
	else{
		PREC invLen = 1.0 / sqrt(v1(1)*v1(1) + v1(2) * v1(2));
		v2 = typename MyMatrix<double>::Vector3(0, v1(2) * invLen, -v1(1) *invLen);
	}
	v3 = v1.cross(v2);

  v2.normalize();
  v3.normalize();
  
};

/* @} */

#endif
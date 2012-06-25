
#ifndef BoxGeometry_hpp
#define BoxGeometry_hpp

#include <Eigen/Dense>

#include "TypeDefs.hpp"


template<class PREC>
class BoxGeometry{
public:

  DEFINE_MATRIX_TYPES
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BoxGeometry(Vector3 center, Vector3 extent):
  m_extent(extent), m_center(center)
  {};

  Vector3 m_extent; ///< Vector of the extend of the box in all directions.
  Vector3 m_center; ///< Vector to the center of the box in body frame! (mostly zero).
};


#endif

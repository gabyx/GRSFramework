
#ifndef PlaneGeometry_hpp
#define PlaneGeometry_hpp

#include <TypeDefs.hpp>

template<class PREC> 
class PlaneGeometry{
public:
  DEFINE_MATRIX_TYPES

  PlaneGeometry( const Vector3 & n, const Vector3 & p ):
  m_normal(n), m_pos(p)
  {}

  Vector3 m_normal; // in K frame
  Vector3 m_pos;    // in K frame

};


#endif

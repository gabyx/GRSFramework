
#ifndef HalfspaceGeometry_hpp
#define HalfspaceGeometry_hpp

#include <TypeDefs.hpp>
#include "PlaneGeometry.hpp"

template<class PREC> 
class HalfspaceGeometry : public PlaneGeometry<PREC>{
public:
  DEFINE_MATRIX_TYPES

  HalfspaceGeometry( const Vector3 & n, const Vector3 & p ): 
  PlaneGeometry<PREC>(n,p)
  {};

};


#endif

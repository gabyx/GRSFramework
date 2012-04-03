
#ifndef SphereGeometry_hpp
#define SphereGeometry_hpp

#include <Eigen/Dense>

template<class PREC> 
class SphereGeometry{
public:
  SphereGeometry(PREC r):
  m_radius(r)
  {};

	PREC m_radius;
};


#endif

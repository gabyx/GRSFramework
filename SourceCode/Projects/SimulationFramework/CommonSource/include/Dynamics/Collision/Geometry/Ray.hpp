#ifndef Ray_hpp
#define Ray_hpp


#include "MyMatrixDefs.hpp"


class Ray {
public:

    DEFINE_MATRIX_TYPES

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Ray(Vector3 p, Vector3 d) :m_p(p), m_d(d.normalize()){
        m_mint = 0.0;
        m_maxt = std::numeric_limits<PREC>::max();
    }

    Ray(void) {
        m_p = Vector3(0.0, 0.0, 0.0);
        m_d = Vector3(0.0, 0.0, 1.0);
        m_mint = 0.0;
        m_maxt = std::numeric_limits<PREC>::max();
    }

    Ray::~Ray(){};

    Vector3 getPointOnRay(double t) const {
        return m_p + (m_d*t);
    }

	Vector3 m_p; ///< start point
	Vector3 m_d; ///< normalized direction vector

	PREC m_mint; ///< min. line parameter t
	PREC m_maxt; ///< max. line parameter t

	// epsilon t to avoid self intersection due to rounding errors
	// if the ray starts exactly on an element
	static const double epsilon_t = 0.001;

};


#endif //Ray_hpp

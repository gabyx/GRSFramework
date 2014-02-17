#ifndef ContactFrame_hpp
#define ContactFrame_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

class ContactFrame{
public:

    DEFINE_MATRIX_TYPES
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ContactFrame(){
        m_e_x.setZero();
        m_e_y.setZero();
        m_e_z.setZero();
        m_p.setZero();
    }
    Vector3 m_e_x; /// e_x in frame I
    Vector3 m_e_y; /// e_y in frame I
    Vector3 m_e_z; /// e_z in frame I
    Vector3 m_p;  /// location of the contact in frame I
};


#endif

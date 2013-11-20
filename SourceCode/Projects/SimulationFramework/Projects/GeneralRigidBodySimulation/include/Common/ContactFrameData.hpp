#ifndef ContactFrameData_hpp
#define ContactFrameData_hpp

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

template<class PREC>
class ContactFrameData{
public:
    Vector3 m_e_x;
    Vector3 m_e_y;
    Vector3 m_e_z;
};

template<class PREC>
class ContactFrameDataVis : public ContactFrameData<PREC> {
public:
    unsigned int id;
};




#endif

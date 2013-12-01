#ifndef RigidBodyMPI_hpp
#define RigidBodyMPI_hpp

#include "RigidBody.hpp"

#include "BodyInfoMap.hpp"

template<typename TRigidBodyConfig >
class RigidBodyMPI: public RigidBodyBase<TRigidBodyConfig> {
public:

    BodyInfo m_pBodyProcessInfo;

};


#endif

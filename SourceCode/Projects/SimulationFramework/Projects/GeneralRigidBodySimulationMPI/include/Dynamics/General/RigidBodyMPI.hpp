#ifndef RigidBodyMPI_hpp
#define RigidBodyMPI_hpp

#include "RigidBody.hpp"

#include "BodyInfoMap.hpp"

class RigidBodyBaseMPI: public RigidBodyBase {
public:

    typedef RigidBodyBase  AbsoluteBaseType; ///< The absolut base type where m_id is defined, for the rigid body container
    typedef BodyProcessInfo BodyInfoType;

    RigidBodyBaseMPI(const RigidBodyIdType & id): RigidBodyBase(id), m_pBodyInfo(NULL){};
    ~RigidBodyBaseMPI(){
        if(m_pBodyInfo){delete m_pBodyInfo; m_pBodyInfo = NULL;}
    };

    BodyInfoType * m_pBodyInfo; ///< This is a class which contains all related info for the mpi information

};


#endif

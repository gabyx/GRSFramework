#ifndef RigidBodyMPI_hpp
#define RigidBodyMPI_hpp

#include "RigidBody.hpp"

#include "BodyInfoMap.hpp"


class RigidBodyBaseMPI: public RigidBodyBase {
public:

    using AbsoluteBaseType = RigidBodyBase ; ///< The absolut base type where m_id is defined, for the rigid body container
    using BodyInfoType = BodyProcessInfo;

    RigidBodyBaseMPI(const RigidBodyIdType & id): RigidBodyBase(id), m_pBodyInfo(nullptr){};
    ~RigidBodyBaseMPI(){
        if(m_pBodyInfo){delete m_pBodyInfo; m_pBodyInfo = nullptr;}
    };

    /**
    This is a class which contains all related info for the mpi information,
    only local and remote bodies have such a type, these get assigned during body communication and in the BodyCommunicator constructor
    */
    BodyInfoType * m_pBodyInfo;


};


#endif

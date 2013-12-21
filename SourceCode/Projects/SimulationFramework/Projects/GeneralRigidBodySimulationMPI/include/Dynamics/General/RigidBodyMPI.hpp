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

    static void changeBodyWeighting(RigidBodyBaseMPI * body, const unsigned int & multiplicity,const PREC & multiplicityFactor){
        ASSERTMSG( body->m_pSolverData->m_multiplicity == 1, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << " has multiplcity : " << body->m_pSolverData->m_multiplicity)
        body->m_h_term *= multiplicityWeight; // h * alpha_i = h_i
        body->m_MassMatrixInv_diag *= 1.0/multiplicityWeight; // M-¹ * 1/alpha_i = M-¹_i
    }

    static void changeToNormalBody(RigidBodyBaseMPI * body){
        ASSERTMSG( body->m_pSolverData->m_multiplicity != 1, "Body id: " << RigidBodyId::getBodyIdString(body->m_id) << " has multiplcity : " << body->m_pSolverData->m_multiplicity)
        body->m_pSolverData->m_multiplicity = 1;
        body->m_pSolverData->m_multiplicityWeight = 1.0;
        body->m_h_term *= 1/multiplicityWeight;  // h_i * 1/alpha_i = h
        body->m_MassMatrixInv_diag *= 1.0/multiplicityWeight;  // M-¹_i * alpha_i = M-¹
    }


};


#endif

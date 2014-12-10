#ifndef GMSF_Dynamics_General_RigidBodyId_hpp
#define GMSF_Dynamics_General_RigidBodyId_hpp

#include <string>
#include <sstream>



class RigidBodyId{

public:
    using Type = uint64_t; ///< [ --- last 32bit is GroupNr --- , --- first 32bit is BodyNr --- ]
    using HalfType = uint32_t;

    template<typename TRigidBodyType >
    inline static HalfType getGroupNr(const TRigidBodyType * body){
        Type id = body->m_id;
        id >>= sizeof(HalfType)*8;
        return reinterpret_cast<HalfType &>(id);
    };

    template<typename TRigidBodyType >
    inline static HalfType getBodyNr(const TRigidBodyType * body){
        return reinterpret_cast<const HalfType &>(body->m_id);
    };

    inline static HalfType getGroupNr(const Type & id){
        Type id2 = id;
        id2 >>= sizeof(HalfType)*8;
        return reinterpret_cast<HalfType &>(id2);
    };

    inline static HalfType getBodyNr(const Type & id){
        return reinterpret_cast<const HalfType &>(id);
    };

    template<typename TRigidBodyType >
    inline static std::string getBodyIdString(const TRigidBodyType * body){
        std::stringstream s;
        s <<"("<< RigidBodyId::getGroupNr(body) << ","<< RigidBodyId::getBodyNr(body) << ")";
        return s.str();
    };

    inline static std::string getBodyIdString(const Type & id){
        std::stringstream s;
        s <<"("<< RigidBodyId::getGroupNr(id) << ","<< RigidBodyId::getBodyNr(id) << ")";
        return s.str();
    };

    inline static Type makeId(  unsigned int groupNr, unsigned int bodyNr){
        Type res = 0;
        res |= (uint64_t)groupNr;
        res <<= 32;
        res |= (uint64_t)bodyNr;
        return res;
    };
};

/** Definition of the RigidBodyId type */
using RigidBodyIdType = RigidBodyId::Type;

/** Definition of the RigidBodyId half type */
using RigidBodyIdHalfType = RigidBodyId::HalfType;

#endif

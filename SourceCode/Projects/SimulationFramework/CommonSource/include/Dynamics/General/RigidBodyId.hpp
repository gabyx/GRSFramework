#ifndef RigidBodyId_hpp
#define RigidBodyId_hpp

#include <string>
#include <sstream>



class RigidBodyId{

public:
    typedef uint64_t Type; ///< [ --- last 32bit is GroupNr --- , --- first 32bit is BodyNr --- ]

    template<typename TRigidBodyType >
    inline static unsigned int getGroupNr(const TRigidBodyType * body){
        Type id = body->m_id;
        id >>= 32;
        return *(reinterpret_cast<unsigned int *>(&(id)));
    };

    template<typename TRigidBodyType >
    inline static unsigned int getBodyNr(const TRigidBodyType * body){
        return *(reinterpret_cast<const unsigned int *>(&(body->m_id)));
    };

    inline static unsigned int getGroupNr(const Type & id){
        Type id2 = id;
        id2 >>= 32;
        return *(reinterpret_cast<unsigned int *>(&(id2)));
    };

    inline static unsigned int getBodyNr(const Type & id){
        return *(reinterpret_cast<const unsigned int *>(&(id)));
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
typedef RigidBodyId::Type RigidBodyIdType;

#endif

#ifndef RigidBodyId_hpp
#define RigidBodyId_hpp

#include <string>
#include <sstream>



class RigidBodyId{

public:
    typedef uint64_t Type;

    template<typename TRigidBodyType >
    static unsigned int getGroupNr(const TRigidBodyType * body){
        Type id = body->m_id;
        id >>= 32;
        return *(reinterpret_cast<unsigned int *>(&(id)));
    };

    template<typename TRigidBodyType >
    static unsigned int getBodyNr(const TRigidBodyType * body){
        return *(reinterpret_cast<const unsigned int *>(&(body->m_id)));
    };

    static unsigned int getGroupNr(const Type & id){
        Type id2 = id;
        id2 >>= 32;
        return *(reinterpret_cast<unsigned int *>(&(id2)));
    };

    static unsigned int getBodyNr(const Type & id){
        return *(reinterpret_cast<const unsigned int *>(&(id)));
    };

    template<typename TRigidBodyType >
    static std::string getBodyIdString(const TRigidBodyType * body){
        std::stringstream s;
        s <<"("<< RigidBodyId::getGroupNr(body) << ","<< RigidBodyId::getBodyNr(body) << ")";
        return s.str();
    };

    static std::string getBodyIdString(const Type & id){
        std::stringstream s;
        s <<"("<< RigidBodyId::getGroupNr(id) << ","<< RigidBodyId::getBodyNr(id) << ")";
        return s.str();
    };

    static Type makeId( unsigned int bodyNr, unsigned int processNr){
        Type res = 0;
        res |= (uint64_t)processNr;
        res <<= 32;
        res |= (uint64_t)bodyNr;
        return res;
    };
};

/** Definition of the RigidBodyId type */
typedef RigidBodyId::Type RigidBodyIdType;

#endif

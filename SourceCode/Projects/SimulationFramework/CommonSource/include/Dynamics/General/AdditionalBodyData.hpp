
#ifndef AdditionalBodyData_hpp
#define AdditionalBodyData_hpp

#include "TypeDefs.hpp"
#include "AssertionDebug.hpp"

#include "EnumClassHelper.hpp"

namespace AdditionalBodyData{

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    enum class TypeEnum : unsigned int {
        NOTHING = 0,
        PROCESS = 1,
        PROCESS_MATERIAL = 2
    };

    class Bytes{
        public:
            const TypeEnum m_type;
            virtual ~Bytes(){};
            Bytes(TypeEnum type): m_type(type){}
    };

    // Type 1
    class Process : public Bytes{
    public:

        static const unsigned int nBytes = sizeof(RankIdType);

        Process(): Bytes( TypeEnum::PROCESS ){}
        ~Process(){}
        RankIdType m_processId;

        template<typename Stream>
        void read(Stream * s){
            *s >> m_processId;
        }

    };

    // Type 2
    class ProcessMaterial : public Bytes{
    public:

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(unsigned int);

        ProcessMaterial(): Bytes(TypeEnum::PROCESS_MATERIAL){}
        ~ProcessMaterial(){}
        RankIdType m_processId;
        unsigned int m_materialId;

        template<typename Stream>
        void read(Stream * s){
            *s >> m_processId;
            *s >> m_materialId;
        }

    };

    inline Bytes * create(TypeEnum type){
        switch(type){
            case TypeEnum::NOTHING:
                return nullptr;
            case TypeEnum::PROCESS:
                return new Process();
            case TypeEnum::PROCESS_MATERIAL:
                return new ProcessMaterial();
        }
    }

    static constexpr std::streamoff getAdditionalBytesPerBody(unsigned int type){
        return (type==EnumConversion::toIntegral(TypeEnum::PROCESS)) ? Process::nBytes :
                 (
                 (type==EnumConversion::toIntegral(TypeEnum::PROCESS_MATERIAL)) ? ProcessMaterial::nBytes  :  0
                 );
    }



}


#endif // AdditionalBodyData_hpp

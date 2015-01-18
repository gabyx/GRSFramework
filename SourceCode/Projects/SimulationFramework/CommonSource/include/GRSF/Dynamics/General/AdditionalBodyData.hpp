
#ifndef GRSF_Dynamics_General_AdditionalBodyData_hpp
#define GRSF_Dynamics_General_AdditionalBodyData_hpp

#include "GRSF/Common/TypeDefs.hpp"
#include "GRSF/Common/AssertionDebug.hpp"

#include RigidBody_INCLUDE_FILE

#include "GRSF/Common/EnumClassHelper.hpp"

namespace AdditionalBodyData{

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    enum class TypeEnum : unsigned int {
        NOTHING = 0,
        PROCESS = 1,
        PROCESS_MATERIAL = 2,
        PROCESS_MATERIAL_OVERLAP = 3,
        PROCESS_OVERLAP = 4
    };

    template<unsigned int Type> class AddBytes;
    template<typename T> struct BytesTraits;

    template<unsigned int Type>
    struct BytesTraits< AddBytes<Type> >{
        static const TypeEnum m_type = static_cast<TypeEnum>(Type);
    };



    #define MAKE_ADDITIONALBODYDATA_FRIEND \
        friend class AdditionalBodyData::AddBytes<1>; \
        friend class AdditionalBodyData::AddBytes<2>; \
        friend class AdditionalBodyData::AddBytes<3>; \
        friend class AdditionalBodyData::AddBytes<4>;

    #define ADDBYTES_SWITCH(_type_ , _function_ , _args_...)  \
        switch( EnumConversion::toIntegral(_type_) ) { \
            case 0: /*Nothing*/ \
                break; \
            case 1: \
                static_cast<AdditionalBodyData::AddBytes<1>* >(this)->_function_( _args_ ); \
                break; \
            case 2: \
                static_cast<AdditionalBodyData::AddBytes<2>* >(this)->_function_( _args_ ); \
                break; \
            case 3: \
                static_cast<AdditionalBodyData::AddBytes<3>* >(this)->_function_( _args_ ); \
                break; \
            case 4: \
                static_cast<AdditionalBodyData::AddBytes<4>* >(this)->_function_( _args_ ); \
                break; \
            default: \
                ERRORMSG("No function call " #_function_ " for type " << EnumConversion::toIntegral(_type_) ); \
        }

    #define ADDBYTES_STATICSWITCH( _type_ , _function_ , _args_...)  \
        switch( EnumConversion::toIntegral(_type_) ) { \
            case 0: /*Nothing*/ \
                break; \
            case 1: \
                AdditionalBodyData::AddBytes<1>::_function_( _args_ ); \
                break; \
            case 2: \
                AdditionalBodyData::AddBytes<2>::_function_( _args_ ); \
                break; \
            case 3: \
                AdditionalBodyData::AddBytes<3>::_function_( _args_ ); \
                break; \
             case 4: \
                AdditionalBodyData::AddBytes<4>::_function_( _args_ ); \
                break; \
            default: \
                ERRORMSG("No function call " #_function_ " for type " << EnumConversion::toIntegral(_type_) ); \
        }


     #define ADDBYTES_VISITOR_CASE_SWITCH( N ) \
            case N: \
                { \
                    visitor( static_cast<AdditionalBodyData::AddBytes<N>* >(this) ); \
                } \
            break; \

    #define ADDBYTES_VISITOR_APPLY_VISITOR_SWITCH\
            switch(EnumConversion::toIntegral(this->m_type)){ \
                case 0: /*Nothing*/ \
                    break; \
                ADDBYTES_VISITOR_CASE_SWITCH(1) \
                ADDBYTES_VISITOR_CASE_SWITCH(2) \
                ADDBYTES_VISITOR_CASE_SWITCH(3) \
                ADDBYTES_VISITOR_CASE_SWITCH(4) \
            default: \
                ERRORMSG("TYPE: "<< EnumConversion::toIntegral(this->m_type)<<" not implemented in switch statement"); \
            }; \

    class Bytes{
        public:
            virtual ~Bytes(){};
            Bytes(TypeEnum type): m_type(type){}

            template<typename Stream>
            void read(Stream & s);

            template<typename Archive, typename TRigidBody >
            void write(Archive & oa, TRigidBody *body);

            template<typename Visitor>
            void applyVisitor(Visitor & visitor);

            const TypeEnum m_type;
    };



    // Type 1
    template<>
    class AddBytes<EnumConversion::toIntegral(TypeEnum::PROCESS)> : public Bytes{
    public:

        static const unsigned int nBytes = sizeof(RankIdType);

        AddBytes(): Bytes( BytesTraits<AddBytes>::m_type ){}
        ~AddBytes(){}
        RankIdType m_processId;

        template<typename Stream>
        void read(Stream & s){
            s >> m_processId;
        }

        template<typename Archive, typename TRigidBody >
        inline static void write(Archive & oa, TRigidBody *body) {
            oa << body->m_pBodyInfo->m_ownerRank; // write owner rank
        }

    };

    // Type 2
    template<>
    class AddBytes<EnumConversion::toIntegral(TypeEnum::PROCESS_MATERIAL)> : public Bytes{
    public:

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(unsigned int);

        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        unsigned int m_materialId;

        template<typename Stream>
        void read(Stream & s){
            s >> m_processId;
            s >> m_materialId;
        }

        template<typename Archive, typename TRigidBody >
        inline static void write(Archive & oa, TRigidBody *body) {
            oa << body->m_pBodyInfo->m_ownerRank; // write owner rank
            oa << body->m_materialId; // write material id
        }
    };

    // Type 3
    template<>
    class AddBytes<EnumConversion::toIntegral(TypeEnum::PROCESS_MATERIAL_OVERLAP)>: public Bytes{
    public:

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(unsigned int) + sizeof(PREC);

        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        typename RigidBodyType::BodyMaterialType m_materialId;
        PREC m_overlapTotal;

        template<typename Stream>
        void read(Stream & s){
            s >> m_processId;
            s >> m_materialId;
            s >> m_overlapTotal;
        }

        template<typename Archive, typename TRigidBody >
        inline static void write(Archive & oa, TRigidBody *body) {
            oa << body->m_pBodyInfo->m_ownerRank; // write owner rank
            oa << body->m_eMaterial; // write material id
            oa << body->m_pSolverData->m_overlapTotal; // write totalOverlap
        }
    };

    // Type 4
    template<>
    class AddBytes<EnumConversion::toIntegral(TypeEnum::PROCESS_OVERLAP)>: public Bytes{
    public:

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(unsigned int) + sizeof(PREC);

        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        PREC m_overlapTotal;

        template<typename Stream>
        void read(Stream & s){
            s >> m_processId;
            s >> m_overlapTotal;
        }

        template<typename Archive, typename TRigidBody >
        inline static void write(Archive & oa, TRigidBody *body) {
            oa << body->m_pBodyInfo->m_ownerRank; // write owner rank
            oa << body->m_pSolverData->m_overlapTotal; // write totalOverlap
        }
    };


    // Implementation Bytes:
    template<typename Stream>
    void Bytes::read(Stream & s){
        ADDBYTES_SWITCH(m_type,read,s);
    }
    template<typename Archive, typename TRigidBody >
    void Bytes::write(Archive & oa, TRigidBody *body){
        ADDBYTES_SWITCH(m_type,write,oa,body);
    }
    template<typename Visitor>
    void Bytes::applyVisitor(Visitor & visitor){
        ADDBYTES_VISITOR_APPLY_VISITOR_SWITCH
    }

    /** Creator function for the AddBytes */
    inline Bytes * create(TypeEnum type){
        switch(EnumConversion::toIntegral(type)){
            case 0:
                return nullptr;
            case 1:
                return new AddBytes<1>();
            case 2:
                return new AddBytes<2>();
            case 3:
                return new AddBytes<3>();
            case 4:
                return new AddBytes<4>();
            default:
                ERRORMSG("This type " << EnumConversion::toIntegral(type) << " construction is not implemented!")
        }
        return nullptr;
    }

    /** Get the number of bytes of a certain AddBytes spezialization*/
    static constexpr std::streamoff getAdditionalBytesPerBody(TypeEnum type){
        return
             (EnumConversion::toIntegral(type)==1) ? AddBytes<1>::nBytes  :
                (
                    (EnumConversion::toIntegral(type)==2) ? AddBytes<2>::nBytes :
                        (
                            (EnumConversion::toIntegral(type)==3) ? AddBytes<3>::nBytes : 0
                        )
                );

    }

    /** Static write function */
    template<typename Archive, typename TRigidBody >
    inline void write(const TypeEnum & type, Archive & oa, TRigidBody *body){
        ADDBYTES_STATICSWITCH(type,write,oa,body);
    }

    /** Static write function */
    template<unsigned int Type, typename Archive, typename TRigidBody >
    inline void write(Archive & oa, TRigidBody *body){
        AddBytes<Type>::write(oa,body);
    }


}


#endif // AdditionalBodyData_hpp

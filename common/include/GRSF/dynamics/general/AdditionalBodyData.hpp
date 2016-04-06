// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_AdditionalBodyData_hpp
#define GRSF_dynamics_general_AdditionalBodyData_hpp

#include "GRSF/common/TypeDefs.hpp"
#include "GRSF/common/AssertionDebug.hpp"

#include RigidBody_INCLUDE_FILE

#include "GRSF/common/EnumClassHelper.hpp"

namespace AdditionalBodyData{

    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES
    DEFINE_MPI_INFORMATION_CONFIG_TYPES

    enum class TypeEnum : unsigned int {
        NOTHING = 0,
        PROCESS = 1,
        PROCESS_MATERIAL = 2,
        PROCESS_MATERIAL_OVERLAP = 3,
        PROCESS_OVERLAP = 4,
        PROCESS_MATERIAL_OVERLAP_GLOBGEOMID = 5
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
        friend class AdditionalBodyData::AddBytes<4>; \
        friend class AdditionalBodyData::AddBytes<5>;

    #define ADDBYTES_SWITCH_CASE(N, _function_ , _args_... )\
            case N: \
                static_cast<AdditionalBodyData::AddBytes<N>* >(this)->_function_( _args_ ); \
                break;

    #define ADDBYTES_SWITCH(_type_ , _function_ , _args_...)  \
        switch( EnumConversion::toIntegral(_type_) ) { \
            case 0: /*Nothing*/ \
                break; \
            ADDBYTES_SWITCH_CASE(1,_function_, _args_) \
            ADDBYTES_SWITCH_CASE(2,_function_, _args_) \
            ADDBYTES_SWITCH_CASE(3,_function_, _args_) \
            ADDBYTES_SWITCH_CASE(4,_function_, _args_) \
            ADDBYTES_SWITCH_CASE(5,_function_, _args_) \
            default: \
                ERRORMSG("No function call " #_function_ " for type " << EnumConversion::toIntegral(_type_) ); \
        }

    #define ADDBYTES_STATICSWITCH_CASE(N, _function_ , _args_...)\
            case N: \
                AdditionalBodyData::AddBytes<N>::_function_( _args_ ); \
                break; \

    #define ADDBYTES_STATICSWITCH( _type_ , _function_ , _args_...)  \
        switch( EnumConversion::toIntegral(_type_) ) { \
            case 0: /*Nothing*/ \
                break; \
            ADDBYTES_STATICSWITCH_CASE(1, _function_ , _args_ ) \
            ADDBYTES_STATICSWITCH_CASE(2, _function_ , _args_ ) \
            ADDBYTES_STATICSWITCH_CASE(3, _function_ , _args_ ) \
            ADDBYTES_STATICSWITCH_CASE(4, _function_ , _args_ ) \
            ADDBYTES_STATICSWITCH_CASE(5, _function_ , _args_ ) \
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
                ADDBYTES_VISITOR_CASE_SWITCH(5) \
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


    // Type 0 , Nothing
    template<>
    class AddBytes<EnumConversion::toIntegral(TypeEnum::NOTHING)> : public Bytes{
        public:
        static const unsigned int nBytes = 0;
        AddBytes(): Bytes( BytesTraits<AddBytes>::m_type ){}
        ~AddBytes(){}
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



        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        typename RigidBodyType::BodyMaterialType  m_materialId;

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(RigidBodyType::BodyMaterialType);

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


        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        typename RigidBodyType::BodyMaterialType m_materialId;
        PREC m_overlapTotal;

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(RigidBodyType::BodyMaterialType) + sizeof(PREC);

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


        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        PREC m_overlapTotal;

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(PREC);

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

    // Type 5
    template<>
    class AddBytes<EnumConversion::toIntegral(TypeEnum::PROCESS_MATERIAL_OVERLAP_GLOBGEOMID)>: public Bytes{
    public:


        AddBytes(): Bytes(BytesTraits<AddBytes>::m_type){}
        ~AddBytes(){}
        RankIdType m_processId;
        typename RigidBodyType::BodyMaterialType m_materialId;
        PREC m_overlapTotal;
        typename RigidBodyType::GlobalGeomIdType m_geomId;

        static const unsigned int nBytes = sizeof(RankIdType) + sizeof(RigidBodyType::BodyMaterialType)
                                           + sizeof(PREC) + sizeof(RigidBodyType::GlobalGeomIdType);

        template<typename Stream>
        void read(Stream & s){
            s >> m_processId;
            s >> m_materialId;
            s >> m_overlapTotal;
            s >> m_geomId;
        }

        template<typename Archive, typename TRigidBody >
        inline static void write(Archive & oa, TRigidBody *body) {
            oa << body->m_pBodyInfo->m_ownerRank; // write owner rank
            oa << body->m_eMaterial; // write material id
            oa << body->m_pSolverData->m_overlapTotal; // write totalOverlap
            oa << body->m_globalGeomId; // write geomId
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
                return new AddBytes<0>();
            case 1:
                return new AddBytes<1>();
            case 2:
                return new AddBytes<2>();
            case 3:
                return new AddBytes<3>();
            case 4:
                return new AddBytes<4>();
            case 5:
                return new AddBytes<5>();
            default:
                ERRORMSG("This type " << EnumConversion::toIntegral(type) << " construction is not implemented!")
        }
        return nullptr;
    }

    /** Construct new additional Data if it does not match */
    inline void createNew(Bytes* & data, TypeEnum type){

        if(data) {
            if(data->m_type != type) {
                delete data;
                data = AdditionalBodyData::create(type);
            }
        } else {
           data = AdditionalBodyData::create(type);
        }
    }


    /** Get the number of bytes of a certain AddBytes spezialization*/
    static constexpr std::streamoff getAdditionalBytesPerBody(TypeEnum type){
        return
             (EnumConversion::toIntegral(type)==0) ? AddBytes<0>::nBytes  :
                (
                    (EnumConversion::toIntegral(type)==1) ? AddBytes<1>::nBytes :
                        (
                            (EnumConversion::toIntegral(type)==2) ? AddBytes<2>::nBytes :
                                (
                                    (EnumConversion::toIntegral(type)==3) ? AddBytes<3>::nBytes :
                                        (
                                            (EnumConversion::toIntegral(type)==4) ? AddBytes<4>::nBytes :
                                                (
                                                    (EnumConversion::toIntegral(type)==5) ? AddBytes<5>::nBytes : 0
                                                 )
                                         )
                                )
                        )
                );

    }

    /** Dynamic write function */
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

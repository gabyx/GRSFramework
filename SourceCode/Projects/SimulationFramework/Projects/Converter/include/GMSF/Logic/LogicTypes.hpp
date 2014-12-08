#ifndef GMSF_Logic_LogicTypes_hpp
#define GMSF_Logic_LogicTypes_hpp

    #include <memory>

    #include <boost/mpl/at.hpp>
    #include <boost/mpl/find.hpp>
    #include <boost/mpl/vector.hpp>
    #include <boost/mpl/joint_view.hpp>

    #include <boost/filesystem.hpp>

    #include "TypeDefs.hpp"

    #include "RenderMaterial.hpp"

    namespace LogicTypes{

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        // The basic types
        using TypeSeqBasic = boost::mpl::vector<double,
                                               float,
                                               bool,
                                               char,
                                               short,
                                               int,
                                               long int,
                                               long long int,
                                               unsigned char,
                                               unsigned short,
                                               unsigned int,
                                               unsigned long int,
                                               unsigned long long int,
                                               std::string,
                                               boost::filesystem::path>;

        // Custom types
        using TypeSeq = boost::mpl::vector<
                                                    double,
                                                    float,
                                                    bool,
                                                    char,
                                                    short,
                                                    int,
                                                    long int,
                                                    long long int,
                                                    unsigned char,
                                                    unsigned short,
                                                    unsigned int,
                                                    unsigned long int,
                                                    unsigned long long int,
                                                    std::string,
                                                    boost::filesystem::path,
                                                    Vector3,
                                                    Quaternion,
                                                    VectorQBody,
                                                    VectorUBody,
                                                    RenderMaterial * >;

//       // The total type sequence!
//       // TypeSequence for the template parameter T in LogicSocket<T>
//       using TypeSeq = typename boost::mpl::joint_view< TypeSeqBasic , TypeSeqCustom>::type;


        #define LOGICSOCKET_CASE_SWITCH( N ) \
            case N: \
                { \
                typedef typename boost::mpl::at_c<TypeSeq, N >::type Type; \
                visitor( castToType<Type>() ); \
                } \
            break; \

        #define LOGICSOCKET_APPLY_VISITOR_SWITCH\
                 \
                switch(this->m_type){ \
                    LOGICSOCKET_CASE_SWITCH(0) \
                    LOGICSOCKET_CASE_SWITCH(1) \
                    LOGICSOCKET_CASE_SWITCH(2) \
                    LOGICSOCKET_CASE_SWITCH(3) \
                    LOGICSOCKET_CASE_SWITCH(4) \
                    LOGICSOCKET_CASE_SWITCH(5) \
                    LOGICSOCKET_CASE_SWITCH(6) \
                    LOGICSOCKET_CASE_SWITCH(7) \
                    LOGICSOCKET_CASE_SWITCH(8) \
                    LOGICSOCKET_CASE_SWITCH(9) \
                    LOGICSOCKET_CASE_SWITCH(10) \
                    LOGICSOCKET_CASE_SWITCH(11) \
                    LOGICSOCKET_CASE_SWITCH(12) \
                    LOGICSOCKET_CASE_SWITCH(13) \
                    LOGICSOCKET_CASE_SWITCH(14) \
                    LOGICSOCKET_CASE_SWITCH(15) \
                    LOGICSOCKET_CASE_SWITCH(16) \
                    LOGICSOCKET_CASE_SWITCH(17) \
                    LOGICSOCKET_CASE_SWITCH(18) \
                    LOGICSOCKET_CASE_SWITCH(19) \
                default: \
                    ERRORMSG("TYPE: "<< this->m_type <<" not implemented in switch statement"); \
                }; \


        //TypesErrorString
        static const char * getTypeName(unsigned int i){

            static const char* types[] = { "double",
                                           "float",
                                           "bool",
                                           "char",
                                           "short",
                                           "int",
                                           "long int",
                                           "long long int",
                                           "unsigned char",
                                           "unsigned short",
                                           "unsigned int",
                                           "unsigned long int",
                                           "unsigned long long int",
                                           "std::string",
                                           "boost::filesystem::path",
                                           "Vector3",
                                           "Quaternion",
                                           "VectorQBody",
                                           "VectorUBody",
                                           "RenderMaterial *"};
            return types[i];
        }
        template<typename T>
        static const char * getTypeName(){
            typedef typename boost::mpl::find<TypeSeq,T>::type iter;
            return getTypeName(iter::pos::value);
        }
    };


    #endif // LogicTypes_hpp


    #ifndef LogicTypes_hpp
    #define LogicTypes_hpp

    #include <memory>
    #include <boost/filesystem.hpp>

    #include "TypeDefs.hpp"

    #include "RenderMaterial.hpp"

    namespace LogicTypes{

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        using TypeSeq = boost::mpl::vector<double,
                                           float,
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

        //TypesErrorString

        static const char * getTypeName(unsigned int i){

            static const char* types[] = { "double",
                                           "float",
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


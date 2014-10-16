    #ifndef LogicTypes_hpp
    #define LogicTypes_hpp

    #include <memory>

    #include "TypeDefs.hpp"

    #include "RenderMaterial.hpp"

    namespace LogicTypes{

        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        using TypeSeq = boost::mpl::vector<double,
                                           float,
                                           char,
                                           short,
                                           int,
                                           unsigned int,
                                           unsigned long int,
                                           unsigned long long int,
                                           std::string,
                                           Vector3,
                                           Quaternion,
                                           VectorQBody,
                                           VectorUBody,
                                           std::shared_ptr<RenderMaterial> >;

    };


    #endif // LogicTypes_hpp


/*
 *  GRSF/Common/TypeDefs.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef GRSF_common_TypeDefs_hpp
#define GRSF_common_TypeDefs_hpp

#include "GRSF/ConfigFiles/ConfigureFile.hpp"
#include "GRSF/dynamics/general/LayoutConfigTypeDefs.hpp"

#define LogicTypes_INCLUDE_FILE                 "GRSF/logic/LogicTypes.hpp"

#define DynamicsSystem_INCLUDE_FILE             "GRSF/dynamics/general/DynamicsSystem.hpp"
class DynamicsSystem;

#define InclusionSolverSettings_INCLUDE_FILE    "GRSF/dynamics/inclusion/InclusionSolverSettings.hpp"
class InclusionSolverSettings;

#define RigidBody_INCLUDE_FILE                  "GRSF/dynamics/general/RigidBody.hpp"
class RigidBodyBase;
#define RigidBodySolverData_INCLUDE_FILE        "GRSF/dynamics/general/RigidBodySolverData.hpp"
class RigidBodySolverDataCONoG;

class RenderData;

//Try to make framework settings simpler:
struct GlobalConfigs{

    // Global definitions used below
    struct MyConfigs{

        using RigidBodyType = RigidBodyBase ;
        using DynamicsSystemType = DynamicsSystem  ;
        using InclusionSolverSettingsType = InclusionSolverSettings;
    };

    struct GeneralConfigs{
         using RandomGeneratorType = std::mt19937;
    };

    struct SystemConfig{
         using DynamicsSystemType = typename MyConfigs::DynamicsSystemType  ;
    };

    struct DynamicSystemConfigs{
        using RigidBodyType = typename MyConfigs::RigidBodyType               ;
        using InclusionSolverSettingsType = typename MyConfigs::InclusionSolverSettingsType ;
    };

    struct RigidBodyConfigs{
        using LayoutConfigType = LayoutConfig<double, GeneralLayout<7,6> >;
        using BodySolverDataType = RigidBodySolverDataCONoG                        ;
    };

    struct MPIInformationConfigs{
        using RankIdType = unsigned int;
    };

    struct RenderConvertDataConfigs{
        using RenderDataType = RenderData;
    };

};


#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    using InclusionSolverSettingsType = typename GlobalConfigs::DynamicSystemConfigs::InclusionSolverSettingsType    ; \
    using DynamicsSystemType = typename GlobalConfigs::SystemConfig::DynamicsSystemType               ; \
    DEFINE_RIGIDBODY_CONFIG_TYPES \



#define DEFINE_RIGIDBODY_CONFIG_TYPES \
    using RigidBodyType = typename GlobalConfigs::DynamicSystemConfigs::RigidBodyType         ; \
    using BodySolverDataType = typename GlobalConfigs::RigidBodyConfigs::BodySolverDataType   ; \
    DEFINE_LAYOUT_CONFIG_TYPES \
    DEFINE_GENERAL_CONFIG_TYPES

#define DEFINE_LAYOUT_CONFIG_TYPES \
    using LayoutConfigType = typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF( GlobalConfigs::RigidBodyConfigs::LayoutConfigType )

#define DEFINE_MATRIX_TYPES \
    using PREC = typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC; \
    DEFINE_MATRIX_TYPES_OF( GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC )


#define DEFINE_GENERAL_CONFIG_TYPES \
    using RandomGenType = typename GlobalConfigs::GeneralConfigs::RandomGeneratorType;

#define DEFINE_MPI_INFORMATION_CONFIG_TYPES \
    using RankIdType = GlobalConfigs::MPIInformationConfigs::RankIdType;


#define DEFINE_RENDERCONVERTERDATA_CONFIG_TYPES \
    using RenderDataType = typename GlobalConfigs::RenderConvertDataConfigs::RenderDataType               ; \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES




#endif





/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef TypeDefs_hpp
#define TypeDefs_hpp

#include "LayoutConfigDefs.hpp"


#define DynamicsSystem_INCLUDE_FILE             "DynamicsSystemConverter.hpp"
class DynamicsSystemConverter;

#define InclusionSolverSettings_INCLUDE_FILE    "InclusionSolverSettings.hpp"
class InclusionSolverSettings;

#define RigidBody_INCLUDE_FILE                  "RigidBody.hpp"
class RigidBodyBase;
#define RigidBodySolverData_INCLUDE_FILE "RigidBodySolverData.hpp"
class RigidBodySolverDataCONoG;


//Try to make framework settings simpler:
struct GlobalConfigs{

    // Global definitions used below
    struct MyConfigs{

        using RigidBodyType = RigidBodyBase ;
        using DynamicsSystemType = DynamicsSystemConverter  ;
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
        using RigidBodySolverDataType = RigidBodySolverDataCONoG                        ;
    };

    struct MPIInformationConfigs{
        using RankIdType = unsigned int;
    };

};


#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    using InclusionSolverSettingsType = typename GlobalConfigs::DynamicSystemConfigs::InclusionSolverSettingsType    ; \
    using DynamicsSystemType = typename GlobalConfigs::SystemConfig::DynamicsSystemType               ; \
    DEFINE_RIGIDBODY_CONFIG_TYPES \



#define DEFINE_RIGIDBODY_CONFIG_TYPES \
    using RigidBodyType = typename GlobalConfigs::DynamicSystemConfigs::RigidBodyType         ; \
    using RigidBodySolverDataType = typename GlobalConfigs::RigidBodyConfigs::RigidBodySolverDataType   ; \
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


struct MyIOFormat{
  static Eigen::IOFormat Matlab;
};

using MeshPREC = double;




#endif





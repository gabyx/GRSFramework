﻿/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef TypeDefs_hpp
#define TypeDefs_hpp

#include "LayoutConfigDefs.hpp"




#define CollisionSolver_INCLUDE_FILE            "CollisionSolverMPI.hpp"
class CollisionSolverMPI;

#define DynamicsSystem_INCLUDE_FILE             "DynamicsSystemMPI.hpp"
class DynamicsSystemMPI;

#define InclusionSolver_INCLUDE_FILE            "InclusionSolverCONoGMPI.hpp"
class InclusionSolverCONoGMPI;

#define RigidBody_INCLUDE_FILE                  "RigidBodyMPI.hpp"
class RigidBodyBaseMPI;

#define RigidBodySolverData_INCLUDE_FILE        "RigidBodySolverDataMPI.hpp"
class RigidBodySolverDataCONoGMPI;

#define TimeStepper_INCLUDE_FILE                "MoreauTimeStepperMPI.hpp"
class MoreauTimeStepperMPI;

#define InclusionSolverSettings_INCLUDE_FILE    "InclusionSolverSettingsMPI.hpp"
class InclusionSolverSettingsMPI;

//Try to make framework settings simpler:
struct GlobalConfigs{

    // Global definitions used below
    struct MyConfigs{

        using RigidBodyType = RigidBodyBaseMPI          ;

        using DynamicsSystemType = DynamicsSystemMPI         ;

        using TimeStepperType = MoreauTimeStepperMPI      ;

        using CollisionSolverType = CollisionSolverMPI        ;

        using InclusionSolverType = InclusionSolverCONoGMPI   ;
        using InclusionSolverSettingsType = InclusionSolverSettingsMPI;

    };

    struct GeneralConfigs{
         using RandomGeneratorType = std::mt19937;
    };


    struct SolverConfigs{
        using TimeStepperType = MyConfigs::TimeStepperType             ;
    };

    struct TimeStepperConfigs{
        using DynamicsSystemType = typename MyConfigs::DynamicsSystemType          ;
        using CollisionSolverType = typename MyConfigs::CollisionSolverType         ;
        using InclusionSolverType = typename MyConfigs::InclusionSolverType         ;
    };

    struct DynamicSystemConfigs{
        using RigidBodyType = typename MyConfigs::RigidBodyType               ;
        using InclusionSolverSettingsType = typename MyConfigs::InclusionSolverSettingsType     ;
    };

    struct RigidBodyConfigs{
        using LayoutConfigType = LayoutConfig<double, GeneralLayout<7,6> >;
        using RigidBodySolverDataType = RigidBodySolverDataCONoGMPI;
    };


    struct InclusionSolverConfigs{
        using DynamicsSystemType = typename MyConfigs::DynamicsSystemType          ;
        using InclusionSolverSettingsType = typename MyConfigs::InclusionSolverSettingsType ;
    };
    struct CollisionSolverConfigs{
        using DynamicsSystemType = typename MyConfigs::DynamicsSystemType          ;
    };

    struct MPIInformationConfigs{
        using RankIdType = unsigned int;
    };


};


#define DEFINE_CONFIG_TYPES \
   DEFINE_SOLVER_CONFIG_TYPES \

#define DEFINE_SOLVER_CONFIG_TYPES \
    DEFINE_TIMESTEPPER_CONFIG_TYPES \

#define DEFINE_TIMESTEPPER_CONFIG_TYPES \
   using TimeStepperType = typename GlobalConfigs::SolverConfigs::TimeStepperType             ; \
   using InclusionSolverType = typename GlobalConfigs::TimeStepperConfigs::InclusionSolverType    ;                 \
   using CollisionSolverType = typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType    ;                 \
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES \
    using CollisionSolverType = typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType    ;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    using InclusionSolverSettingsType = typename GlobalConfigs::InclusionSolverConfigs::InclusionSolverSettingsType    ; \
    using DynamicsSystemType = typename GlobalConfigs::TimeStepperConfigs::DynamicsSystemType                 ; \
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

#define DEFINE_MPI_INFORMATION_CONFIG_TYPES \
    using RankIdType = GlobalConfigs::MPIInformationConfigs::RankIdType;


#define DEFINE_GENERAL_CONFIG_TYPES \
    using RandomGenType = typename GlobalConfigs::GeneralConfigs::RandomGeneratorType;


struct MyIOFormat{
  static Eigen::IOFormat Matlab;
};

using MeshPREC = double;




#endif




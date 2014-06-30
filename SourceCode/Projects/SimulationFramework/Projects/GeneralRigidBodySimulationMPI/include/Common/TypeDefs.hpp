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




#define CollisionSolver_INCLUDE_FILE            "CollisionSolverMPI.hpp"
class CollisionSolver;

#define DynamicsSystem_INCLUDE_FILE             "DynamicsSystemMPI.hpp"
class DynamicsSystem;

#define InclusionSolver_INCLUDE_FILE            "InclusionSolverCONoGMPI.hpp"
//class InclusionSolverCO;
class InclusionSolverCONoG;

#define RigidBody_INCLUDE_FILE                  "RigidBodyMPI.hpp"
class RigidBodyBaseMPI;

#define RigidBodySolverData_INCLUDE_FILE        "RigidBodySolverDataMPI.hpp"
class RigidBodySolverDataCONoGMPI;

#define TimeStepper_INCLUDE_FILE                "MoreauTimeStepperMPI.hpp"
class MoreauTimeStepper;

#define InclusionSolverSettings_INCLUDE_FILE    "InclusionSolverSettingsMPI.hpp"
class InclusionSolverSettings;

//Try to make framework settings simpler:
namespace GlobalConfigs{

    // Global definitions used below
    namespace MyConfigs{

        typedef RigidBodyBaseMPI        RigidBodyType;

        typedef DynamicsSystem          DynamicsSystemType;

        typedef MoreauTimeStepper       TimeStepperType;

        typedef CollisionSolver         CollisionSolverType;

        typedef InclusionSolverCONoG    InclusionSolverType;
        typedef InclusionSolverSettings InclusionSolverSettingsType;

    };

    namespace GeneralConfigs{
         typedef std::mt19937 RandomGeneratorType;
    };


    namespace SolverConfigs{
        typedef MyConfigs::TimeStepperType              TimeStepperType;
    };

    namespace TimeStepperConfigs{
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
        typedef typename MyConfigs::CollisionSolverType          CollisionSolverType;
        typedef typename MyConfigs::InclusionSolverType          InclusionSolverType;
    };

    namespace DynamicSystemConfigs{
        typedef typename MyConfigs::RigidBodyType                RigidBodyType;
        typedef typename MyConfigs::InclusionSolverSettingsType      InclusionSolverSettingsType;
    };

    namespace RigidBodyConfigs{
        typedef LayoutConfig<double, GeneralLayout<7,6> > LayoutConfigType;
        typedef RigidBodySolverDataCONoGMPI RigidBodySolverDataType;
    }


    namespace InclusionSolverConfigs{
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
        typedef typename MyConfigs::InclusionSolverSettingsType  InclusionSolverSettingsType;
    };
    namespace CollisionSolverConfigs{
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
    };

    namespace MPIInformationConfigs{
        typedef unsigned int RankIdType;
    };


};


#define DEFINE_CONFIG_TYPES \
   DEFINE_SOLVER_CONFIG_TYPES \

#define DEFINE_SOLVER_CONFIG_TYPES \
    DEFINE_TIMESTEPPER_CONFIG_TYPES \

#define DEFINE_TIMESTEPPER_CONFIG_TYPES \
   typedef typename GlobalConfigs::SolverConfigs::TimeStepperType              TimeStepperType; \
   typedef typename GlobalConfigs::TimeStepperConfigs::InclusionSolverType     InclusionSolverType;                 \
   typedef typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES \
    typedef typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    typedef typename GlobalConfigs::InclusionSolverConfigs::InclusionSolverSettingsType     InclusionSolverSettingsType; \
    typedef typename GlobalConfigs::TimeStepperConfigs::DynamicsSystemType                  DynamicsSystemType; \
    DEFINE_RIGIDBODY_CONFIG_TYPES \


#define DEFINE_RIGIDBODY_CONFIG_TYPES \
    typedef typename GlobalConfigs::DynamicSystemConfigs::RigidBodyType          RigidBodyType; \
    typedef typename GlobalConfigs::RigidBodyConfigs::RigidBodySolverDataType    RigidBodySolverDataType; \
    DEFINE_LAYOUT_CONFIG_TYPES \
    DEFINE_GENERAL_CONFIG_TYPES

#define DEFINE_LAYOUT_CONFIG_TYPES \
    typedef typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType LayoutConfigType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF( GlobalConfigs::RigidBodyConfigs::LayoutConfigType )

#define DEFINE_MATRIX_TYPES \
    typedef typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC PREC; \
    DEFINE_MATRIX_TYPES_OF( GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC )

#define DEFINE_MPI_INFORMATION_CONFIG_TYPES \
    typedef GlobalConfigs::MPIInformationConfigs::RankIdType RankIdType;


#define DEFINE_GENERAL_CONFIG_TYPES \
    typedef typename GlobalConfigs::GeneralConfigs::RandomGeneratorType RandomGenType;


struct MyIOFormat{
  static Eigen::IOFormat Matlab;
};

typedef double MeshPREC;




#endif




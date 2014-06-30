/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel Nützi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef TypeDefs_hpp
#define TypeDefs_hpp

#include <random>

#include "LayoutConfigDefs.hpp"



/// Forward declarations , includes and the GlobalConfig typed need to match!


#define StatePool_INCLUDE_FILE "StatePoolVisBackFront.hpp"
class StatePoolVisBackFront;

#define CollisionSolver_INCLUDE_FILE "CollisionSolver.hpp"
class CollisionSolver;

#define DynamicsSystem_INCLUDE_FILE "DynamicsSystem.hpp"
class DynamicsSystem;
#define InclusionSolverSettings_INCLUDE_FILE    "InclusionSolverSettings.hpp"
class InclusionSolverSettings;

//#define InclusionSolver_INCLUDE_FILE "InclusionSolverCO.hpp"
//class InclusionSolverCO;
#define InclusionSolver_INCLUDE_FILE "InclusionSolverCONoG.hpp"
class InclusionSolverCONoG;

#define RigidBody_INCLUDE_FILE "RigidBody.hpp"
class RigidBodyBase;

#define RigidBodySolverData_INCLUDE_FILE "RigidBodySolverData.hpp"
class RigidBodySolverDataCONoG;

#define TimeStepper_INCLUDE_FILE "MoreauTimeStepper.hpp"
class MoreauTimeStepper;




//Try to make framework settings simpler:
namespace GlobalConfigs {

    namespace MyConfigs {

        typedef RigidBodyBase           RigidBodyType;

        typedef DynamicsSystem          DynamicsSystemType;

        typedef MoreauTimeStepper       TimeStepperType;

        typedef CollisionSolver         CollisionSolverType;

        typedef InclusionSolverCONoG    InclusionSolverType;
        typedef InclusionSolverSettings InclusionSolverSettingsType;

    };

    namespace GeneralConfigs{
         typedef std::mt19937 RandomGeneratorType;
    };

    namespace SolverConfigs {
        typedef MyConfigs::TimeStepperType                       TimeStepperType;
    };

    namespace TimeStepperConfigs {
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
        typedef typename MyConfigs::CollisionSolverType          CollisionSolverType;
        typedef typename MyConfigs::InclusionSolverType          InclusionSolverType;
        typedef StatePoolVisBackFront                            StatePoolType;
    };

    namespace DynamicSystemConfigs {
        typedef typename MyConfigs::RigidBodyType                RigidBodyType;
        typedef typename MyConfigs::InclusionSolverSettingsType  InclusionSolverSettingsType;
    };

    namespace RigidBodyConfigs {
        typedef LayoutConfig<double, GeneralLayout<7,6> >        LayoutConfigType;
        typedef RigidBodySolverDataCONoG                         RigidBodySolverDataType;
    }


    namespace InclusionSolverConfigs {
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
        typedef typename MyConfigs::InclusionSolverSettingsType  InclusionSolverSettingsType;
    };
    namespace CollisionSolverConfigs {
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
    };

};




#define DEFINE_CONFIG_TYPES \
   DEFINE_SOLVER_CONFIG_TYPES \

#define DEFINE_SOLVER_CONFIG_TYPES \
    DEFINE_TIMESTEPPER_CONFIG_TYPES \

#define DEFINE_TIMESTEPPER_CONFIG_TYPES \
   typedef typename GlobalConfigs::SolverConfigs::TimeStepperType              TimeStepperType; \
   typedef typename GlobalConfigs::TimeStepperConfigs::StatePoolType           StatePoolType;                 \
   typedef typename GlobalConfigs::TimeStepperConfigs::InclusionSolverType     InclusionSolverType;                 \
   typedef typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES \
    typedef typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    typedef typename GlobalConfigs::DynamicSystemConfigs::InclusionSolverSettingsType     InclusionSolverSettingsType; \
    typedef typename GlobalConfigs::TimeStepperConfigs::DynamicsSystemType                DynamicsSystemType; \
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

#define DEFINE_GENERAL_CONFIG_TYPES \
    typedef typename GlobalConfigs::GeneralConfigs::RandomGeneratorType RandomGenType;

struct MyIOFormat {
    static Eigen::IOFormat Matlab;
};

typedef double MeshPREC;




#endif




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


class StatePoolVisBackFront;
class CollisionSolver;
class DynamicsSystem;
class InclusionSolverCO;
class InclusionSolverCONoG;
class RigidBodyBase;
class RigidBodySolverDataCONoG;
class MoreauTimeStepper;


#define RigidBody_INCLUDE_FILE "RigidBody.hpp"
#define TimeStepper_INCLUDE_FILE "MoreauTimeStepper.hpp"
#define DynamicsSystem_INCLUDE_FILE "DynamicsSystem.hpp"
#define InclusionSolver_INCLUDE_FILE "InclusionSolverCONoG.hpp"
#define CollisionSolver_INCLUDE_FILE "CollisionSolver.hpp"
#define StatePool_INCLUDE_FILE "StatePoolVisBackFront.hpp"
#define RigidBodySolverData_INCLUDE_FILE "RigidBodySolverData.hpp"

//Try to make framework settings simpler:
namespace GlobalConfigs{

    namespace MyConfigs{
        typedef RigidBodyBase           RigidBodyType;
        typedef DynamicsSystem          DynamicsSystemType;
        typedef MoreauTimeStepper       TimeStepperType;
        typedef CollisionSolver         CollisionSolverType;
        typedef InclusionSolverCONoG    InclusionSolverType;
    };

    namespace RigidBodyConfigs{
        typedef LayoutConfig<double, DynamicLayout<7,6> > LayoutConfigType;
        typedef RigidBodySolverDataCONoG RigidBodySolverDataType;
    }
    namespace DynamicSystemConfigs{
        typedef typename MyConfigs::RigidBodyType RigidBodyType;
    };
    namespace InclusionSolverConfigs{
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
    };
    namespace CollisionSolverConfigs{
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
    };

    namespace TimeStepperConfigs{
        typedef typename MyConfigs::DynamicsSystemType           DynamicsSystemType;
        typedef typename MyConfigs::CollisionSolverType          CollisionSolverType;
        typedef typename MyConfigs::InclusionSolverType          InclusionSolverType;
        typedef StatePoolVisBackFront                   StatePoolType;
    };

    namespace SolverConfigs{
        typedef MyConfigs::TimeStepperType              TimeStepperType;
    };


};


#define DEFINE_CONFIG_TYPES \
   DEFINE_SOLVER_CONFIG_TYPES \

#define DEFINE_SOLVER_CONFIG_TYPES \
   typedef typename GlobalConfigs::SolverConfigs::TimeStepperType TimeStepperType; \
   DEFINE_TIMESTEPPER_CONFIG_TYPES \

#define DEFINE_TIMESTEPPER_CONFIG_TYPES \
   typedef typename GlobalConfigs::TimeStepperConfigs::StatePoolType          StatePoolType;                 \
   typedef typename GlobalConfigs::TimeStepperConfigs::InclusionSolverType     InclusionSolverType;                 \
   typedef typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES \
    typedef typename GlobalConfigs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    typedef typename GlobalConfigs::TimeStepperConfigs::DynamicsSystemType     DynamicsSystemType; \
    DEFINE_RIGIDBODY_CONFIG_TYPES \


#define DEFINE_RIGIDBODY_CONFIG_TYPES \
    typedef typename GlobalConfigs::DynamicSystemConfigs::RigidBodyType RigidBodyType; \
    typedef typename GlobalConfigs::RigidBodyConfigs::RigidBodySolverDataType RigidBodySolverDataType; \
    DEFINE_LAYOUT_CONFIG_TYPES\

#define DEFINE_LAYOUT_CONFIG_TYPES \
    typedef typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType LayoutConfigType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF( GlobalConfigs::RigidBodyConfigs::LayoutConfigType )

#define DEFINE_MATRIX_TYPES \
    typedef typename GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC PREC; \
    DEFINE_MATRIX_TYPES_OF( GlobalConfigs::RigidBodyConfigs::LayoutConfigType::PREC )

struct MyIOFormat{
  static Eigen::IOFormat Matlab;
};

typedef double MeshPREC;




#endif




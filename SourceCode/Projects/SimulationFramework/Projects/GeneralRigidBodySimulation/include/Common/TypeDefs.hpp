/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef TypeDefs_hpp
#define TypeDefs_hpp

#include "LayoutConfigDefs.hpp"


/**
* @ingroup Common
* @defgroup TypeDefs Type Definitions for Templates
* @brief This is the definition file for all template names in this framework.
* It defines all template arguments. It allows to have one template argument for a class
* and access through the structure of Config or LayoutConfig different template parameters which
* are already set in this file.
* - \b Config: This is the overall config struct which contains:
* 		- \b TLayoutConfig: This is struct which contains:
*			- \b TReal: The type for the precision (double, float)
*			- \b TLayout:
*				The layout can be choosed statically or dynamically. Dynamically means that NDOFq and NDOFu = -1 which is used for theater
*				matrix library which then creates dynamical allocated space. Normally we use a Dynamic layout because we dont know yet how many bodies
*				are simulated.
*				- \b NDOFq: The integer specifying the DOF of generalized coordinates of the whole system.
*				- \b NDOFu: The integer specifying the DOF of generalized velocities of the whole system.
*				- \b NDOFqObj: The integer specifying the DOF of generalized coordinates of one rigid body.
*				- \b NDOFuObj: The integer specifying the DOF of generalized velocities of one rigid body.
*				- \b NDOFFriction: The integer specifying the DOF of the friction model.
*			-  Matrix definitions...
*		- \b TSolverConfig: This is the solver config which contains:
*			- \b TTimeStepper
*			- \b TInclusionSolver
*			- \b TColissionSolver
*		- \b TSystem: This specifies the system which is simulated.
*/
/* @{ */

// ================================================================================================

/**
* @brief This is the Config (base) which includes a Layout, a Solver, and a System
*/
template< typename _TSolverConfig >
struct Config{
   typedef _TSolverConfig SolverConfigType;
};


/**
* @brief A dynamic double layout specialization. For rigid bodies with NDOFq = 7 , NDOFu = 6 and NDOFFriction = 2.
*/
typedef LayoutConfig<
   double,
   DynamicLayout<7,6>
> DoubleDynamicLayout;


// ================================================================================================
/**
* @brief The solver config with TimeStepper, CollisionSolver and InclusionSolver
*/
template < typename _TTimeStepper>
struct SolverConfig{
   typedef _TTimeStepper TimeStepperType;
};

template < typename _TDynamicsSystem, typename _TCollisionSolver, typename _TInclusionSolver, typename _TStatePool>
struct ConfigTimeStepper{
    typedef _TDynamicsSystem DynamicsSystemType;
    typedef _TCollisionSolver CollisionSolverType;
    typedef _TInclusionSolver InclusionSolverType;
    typedef _TStatePool StatePoolType;
};

template < typename _TRigidBody>
struct ConfigDynamicsSystem{
    typedef _TRigidBody RigidBodyType;
};

template <typename _TDynamicsSystem>
struct ConfigCollisionSolver{
    typedef _TDynamicsSystem  DynamicsSystemType;
};

template <typename _TDynamicsSystem, typename _TCollisionSolver>
struct ConfigInclusionSolver{
    typedef _TDynamicsSystem  DynamicsSystemType;
    typedef _TCollisionSolver CollisionSolverType;
};


template < typename _TLayoutConfig, typename _TRigidBodySolverData>
struct ConfigRigidBody{
    typedef _TLayoutConfig LayoutConfigType;
    typedef _TRigidBodySolverData   RigidBodySolverDataType;
};

// ================================================================================================
/**
* \defgroup SpheresSystemDefinitions Spheres System Definitions
* @brief This is the full specialization for the SphereSystem.
* - \b SpheresConfig: :
* 		- \b DoubleDynamicLayout:
*			- \b TReal: double
*			- \b TLayout:
*				- \b NDOFq: -1
*				- \b NDOFu: -1
*				- \b NDOFqObj: 7
*				- \b NDOFuObj: 6
*				- \b NDOFFriction: 2
*			-  Matrix definitions...
*		- \b SpheresSolverConfig:
*			- \b TTimeStepper: MoreauTimeStepper< 	DoubleDynamicLayout,
*      												SpheresSystem,
*      												CollisionSolver<DoubleDynamicLayout>,
*      												InclusionSolverNT<DoubleDynamicLayout,SpheresSystem,CollisionSolver<DoubleDynamicLayout> >,
*     												StatePoolVisBackFront<DoubleDynamicLayout>
*			- \b TColissionSolver: 	CollisionSolver<DoubleDynamicLayout>
*			- \b InclusionSolver: 	InclusionSolverNT<DoubleDynamicLayout,SpheresSystem,CollisionSolver<DoubleDynamicLayout> >
*		- \b TSystem: SpheresSystem
*/
/* @{ */



template< typename TLayoutConfig > class StatePoolVisBackFront;

template< typename TCollisionSolverConfig > class CollisionSolver;
template< typename TDynamicsSystemConfig  > class DynamicsSystem;

template< typename TInclusionSolverConfig > class InclusionSolverCO;
template< typename TInclusionSolverConfig > class InclusionSolverCONoG;


template< typename TRigidBodyConfig> class RigidBodyBase;
template< typename TLayoutConfig > class RigidBodySolverDataCONoG;
template< typename TLayoutConfig > class RigidBodySolverDataNone;

template< typename TConfigTimeStepper > class MoreauTimeStepper;


// Just defintion to save it!
//typedef SolverConfig
//   <
//   MoreauTimeStepper<
//      DoubleDynamicLayout,
//      SpheresSystem,
//      CollisionSolver<DoubleDynamicLayout>,
//      InclusionSolverNT<DoubleDynamicLayout,SpheresSystem,CollisionSolver<DoubleDynamicLayout>>,
//      StatePoolVisBackFront<DoubleDynamicLayout>
//      >,
//      CollisionSolver<DoubleDynamicLayout>,
//      InclusionSolverNT<DoubleDynamicLayout,SpheresSystem,CollisionSolver<DoubleDynamicLayout> >
//   > GeneralSolverConfigNotOrdered;

// This one is used!!


typedef ConfigRigidBody< DoubleDynamicLayout, RigidBodySolverDataCONoG<DoubleDynamicLayout> > MyRigidBodyConfig;

typedef RigidBodyBase< MyRigidBodyConfig > MyRigidBody; //Define the Class

typedef ConfigDynamicsSystem< MyRigidBody> MyConfigDynamicsSystem;

typedef DynamicsSystem<MyConfigDynamicsSystem> MyDynamicsSystem; //Define the Class

typedef ConfigCollisionSolver< MyDynamicsSystem> MyConfigCollisionSolver;
typedef CollisionSolver<MyConfigCollisionSolver> MyCollisionSolver; //Define the Class

typedef ConfigInclusionSolver<MyDynamicsSystem, MyCollisionSolver>  MyConfigInclusionSolver;
typedef InclusionSolverCONoG< MyConfigInclusionSolver >  MyInclusionSolver; //Define the Class

typedef ConfigTimeStepper<
             MyDynamicsSystem,
             MyCollisionSolver,
             MyInclusionSolver,
             StatePoolVisBackFront<DoubleDynamicLayout>
          > MyConfigTimeStepper;

typedef MoreauTimeStepper< MyConfigTimeStepper > MyMoreauTimeStepper; //Define the Class


typedef SolverConfig<MyMoreauTimeStepper> MySolverConfig;

typedef Config< MySolverConfig > GeneralConfig;

/* @} */

// Macros ==================================================================================================================

/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argurment <typename Config>
* It is used to access all typedefs very easy and gives neat code!
*/
#define DEFINE_CONFIG_TYPES_OF( _ConfigName_ ) \
   DEFINE_SOLVER_CONFIG_TYPES_OF( _ConfigName_::SolverConfigType ) \

#define DEFINE_SOLVER_CONFIG_TYPES_OF( _SolverConfigName_ ) \
   typedef typename _SolverConfigName_::TimeStepperType TimeStepperType; \
   DEFINE_TIMESTEPPER_CONFIG_TYPES_OF( _SolverConfigName_::TimeStepperType::TimeStepperConfigType ) \

#define DEFINE_TIMESTEPPER_CONFIG_TYPES_OF( _TimeStepperConfigName_ ) \
   typedef typename _TimeStepperConfigName_::StatePoolType          StatePoolType;                 \
   typedef typename _TimeStepperConfigName_::DynamicsSystemType     DynamicsSystemType;                 \
   typedef typename _TimeStepperConfigName_::InclusionSolverType     InclusionSolverType;                 \
   typedef typename _TimeStepperConfigName_::CollisionSolverType     CollisionSolverType;                 \
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF ( _TimeStepperConfigName_::DynamicsSystemType ) \

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES_OF( _InclusionSolverConfigName_ ) \
    typedef typename _InclusionSolverConfigName_::DynamicsSystemType     DynamicsSystemType;                 \
    typedef typename _InclusionSolverConfigName_::CollisionSolverType     CollisionSolverType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF( DynamicsSystemType::DynamicsSystemConfig ) \

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES_OF( _CollisionSolverConfigName_ ) \
    typedef typename _CollisionSolverConfigName_::DynamicsSystemType     DynamicsSystemType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF( DynamicsSystemType::DynamicsSystemConfig ) \

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF( _DynamicsSystemConfigName_ ) \
    typedef typename _DynamicsSystemConfigName_::RigidBodyType RigidBodyType; \
    DEFINE_RIGIDBODY_CONFIG_TYPES_OF( _DynamicsSystemConfigName_::RigidBodyType::RigidBodyConfigType ) \


#define DEFINE_RIGIDBODY_CONFIG_TYPES_OF( _RigidBodyConfigName_ ) \
    typedef typename _RigidBodyConfigName_::LayoutConfigType LayoutConfigType; \
    typedef typename _RigidBodyConfigName_::RigidBodySolverDataType RigidBodySolverDataType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF( _RigidBodyConfigName_::LayoutConfigType ) \



/**
* @brief This is the format for the output of matrices.
*/
struct MyIOFormat{
  static Eigen::IOFormat Matlab;
};



typedef double MeshPREC;

/* @} */



//Try to make framework settings simpler:
namespace GlobalDefs{

    namespace RigidBodyConfigs{
        typedef LayoutConfig<double, DynamicLayout<7,6> > LayoutConfigType;
        typedef RigidBodySolverDataCONoG RigidBodySolverDataType;
    }
    namespace DynamicSystemConfigs{
        typedef MyConfigs::RigidBodyType RigidBodyType;
    };
    namespace InclusionSolverConfigs{
        typedef MyConfigs::DynamicsSystemType           DynamicsSystemType;
    };
    namespace CollisionSolverConfigs{
        typedef MyConfigs::DynamicsSystemType           DynamicsSystemType;
    };

    namespace TimeStepperConfigs{
        typedef MyConfigs::DynamicsSystemType           DynamicsSystemType;
        typedef MyConfigs::CollisionSolverType          CollisionSolverType;
        typedef MyConfigs::InclusionSolverType          InclusionSolverType;
        typedef StatePoolVisBackFront                   StatePoolType;
    };

    namespace SolverConfigs{
        typedef MyConfigs::TimeStepperType              TimeStepperType;
    };

    namespace MyConfigs{
        typedef RigidBodyBase           RigidBodyType;
        typedef DynamicsSystem          DynamicsSystemType;
        typedef MoreauTimeStepper       TimeStepperType;
        typedef CollisionSolver         CollisionSolverType;
        typedef InclusionSolverCONoG    InclusionSolverType
    }
};


#define DEFINE_CONFIG_TYPES \
   DEFINE_SOLVER_CONFIG_TYPES \

#define DEFINE_SOLVER_CONFIG_TYPES \
   typedef typename GlobalDefs::SolverConfigs::TimeStepperType TimeStepperType; \
   DEFINE_TIMESTEPPER_CONFIG_TYPES \

#define DEFINE_TIMESTEPPER_CONFIG_TYPES \
   typedef typename GlobalDefs::TimeStepperConfigs::StatePoolType          StatePoolType;                 \
   typedef typename GlobalDefs::TimeStepperConfigs::DynamicsSystemType     DynamicsSystemType;                 \
   typedef typename GlobalDefs::TimeStepperConfigs::InclusionSolverType     InclusionSolverType;                 \
   typedef typename GlobalDefs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_INCLUSIONS_SOLVER_CONFIG_TYPES \
    typedef typename GlobalDefs::TimeStepperConfigs::DynamicsSystemType      DynamicsSystemType;                 \
    typedef typename GlobalDefs::TimeStepperConfigs::CollisionSolverType     CollisionSolverType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_COLLISION_SOLVER_CONFIG_TYPES \
    typedef typename GlobalDefs::TimeStepperConfigs::DynamicsSystemType     DynamicsSystemType;                 \
    DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES \
    typedef typename GlobalDefs::DynamicSystemConfigs::RigidBodyType RigidBodyType; \
    DEFINE_RIGIDBODY_CONFIG_TYPES \


#define DEFINE_RIGIDBODY_CONFIG_TYPES \
    typedef typename GlobalDefs::RigidBodyConfigs::LayoutConfigType LayoutConfigType; \
    typedef typename GlobalDefs::RigidBodyConfigs::RigidBodySolverDataType RigidBodySolverDataType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF( LayoutConfigType ) \





#endif




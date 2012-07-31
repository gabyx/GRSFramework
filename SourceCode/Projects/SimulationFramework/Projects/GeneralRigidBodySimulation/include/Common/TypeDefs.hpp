/*
 *  TypeDefs.hpp
 *
 *  Created by Gabriel NÃ¼tzi on 21.03.10.
 *  Copyright 2010 ETH. All rights reserved.
 *
 */

#ifndef TypeDefs_hpp
#define TypeDefs_hpp


#include <Eigen/Dense>

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/
template<typename TPREC>
struct MyMatrix{
   typedef TPREC PREC;
   //Static assigned Matrices
   typedef Eigen::Matrix<PREC, 4, 4> Matrix44;
   typedef Eigen::Matrix<PREC, 4, 3> Matrix43;
   typedef Eigen::Matrix<PREC, 3, 4> Matrix34;
   typedef Eigen::Matrix<PREC, 3, 3> Matrix33;
   typedef Eigen::Matrix<PREC, 3, 1> Vector3;
   typedef Eigen::Matrix<PREC, 4, 1> Quaternion;
   typedef Eigen::Matrix<PREC, 4, 1> Vector4;
   typedef Eigen::Matrix<PREC, Eigen::Dynamic , 1 >                    VectorDyn;
   typedef Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic >       MatrixDyn;
   typedef Eigen::DiagonalMatrix<PREC, Eigen::Dynamic >                MatrixDiagDyn;
   typedef Eigen::Matrix<PREC, Eigen::Dynamic , Eigen::Dynamic, Eigen::RowMajor> MatrixDynRow;
};
// ================================================================================================
/**
* @brief This is the LayoutConfig which specifies the layout of the system.
*/
template <typename TPREC, typename TLayout>
struct LayoutConfig{
   typedef TPREC PREC;
   typedef TLayout LayoutType;

   // Dynamic or Static Vectors/Matrices
   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, LayoutType::NDOFq>           MatrixQQ;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, LayoutType::NDOFu>           MatrixUU;
   typedef Eigen::DiagonalMatrix<PREC, LayoutType::NDOFq>                  MatrixDiagQQ;
   typedef Eigen::DiagonalMatrix<PREC, LayoutType::NDOFu>                  MatrixDiagUU;

   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, LayoutType::NDOFu>           MatrixQU;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, LayoutType::NDOFq>           MatrixUQ;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, Eigen::Dynamic >         MatrixUDyn;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, Eigen::Dynamic >         MatrixQDyn;

   // Dynamic or Static  Vectors/Matrices
   typedef Eigen::Matrix<PREC, LayoutType::NDOFq, 1>                       VectorQ;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFu, 1>                       VectorU;

   // Static  Vectors/Matrices
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, LayoutType::NDOFuObj>     MatrixQObjUObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFqObj, 1>                    VectorQObj;
   typedef Eigen::Matrix<PREC, LayoutType::NDOFuObj, 1>                    VectorUObj;

   // Static Vectors/Matrices
   typedef typename MyMatrix< PREC >::Matrix44 Matrix44;
   typedef typename MyMatrix< PREC >::Matrix33 Matrix33;
   typedef typename MyMatrix< PREC >::Matrix43 Matrix43;
   typedef typename MyMatrix< PREC >::Matrix34 Matrix34;
   typedef typename MyMatrix< PREC >::Vector3 Vector3;
   typedef typename MyMatrix< PREC >::Vector4 Vector4;
   typedef typename MyMatrix< PREC >::Quaternion Quaternion;
   typedef typename MyMatrix< PREC >::VectorDyn VectorDyn;
   typedef typename MyMatrix< PREC >::MatrixDyn MatrixDyn;
   typedef typename MyMatrix< PREC >::MatrixDiagDyn MatrixDiagDyn;
   typedef typename MyMatrix< PREC >::MatrixDynRow MatrixDynRow;

};


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
* @brief A dynamic layout specialization.
*/
template <int nDOFqObj, int nDOFuObj>
struct DynamicLayout {
   static int const NDOFq = -1;
   static int const NDOFu = -1;
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
};

/**
* @brief A static layout specialization.
*/
template <int NObjects, int nDOFqObj, int nDOFuObj>
struct StaticLayout {
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
   static int const NDOFq = NObjects*NDOFqObj;
   static int const NDOFu = NObjects*NDOFuObj;
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

template < typename _TRigidBody>
struct ConfigCollisionSolver{
    typedef _TRigidBody RigidBodyType;
};

template <typename _TDynamicsSystem, typename _TCollisionSolver,  typename _TRigidBody>
struct ConfigInclusionSolver{
    typedef _TRigidBody       RigidBodyType;
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

typedef ConfigCollisionSolver< MyRigidBody> MyConfigCollisionSolver;
typedef CollisionSolver<MyConfigCollisionSolver> MyCollisionSolver; //Define the Class

typedef ConfigInclusionSolver<MyDynamicsSystem, MyCollisionSolver, MyRigidBody>  MyConfigInclusionSolver;
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
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define DEFINE_MATRIX_TYPES \
   typedef typename MyMatrix< PREC >::Matrix44 Matrix44; \
   typedef typename MyMatrix< PREC >::Matrix33 Matrix33; \
   typedef typename MyMatrix< PREC >::Matrix43 Matrix43; \
   typedef typename MyMatrix< PREC >::Matrix34 Matrix34; \
   typedef typename MyMatrix< PREC >::Vector3 Vector3;   \
   typedef typename MyMatrix< PREC >::Vector4 Vector4;   \
   typedef typename MyMatrix< PREC >::Quaternion Quaternion; \
   typedef typename MyMatrix< PREC >::VectorDyn VectorDyn; \
   typedef typename MyMatrix< PREC >::MatrixDyn MatrixDyn; \
   typedef typename MyMatrix< PREC >::MatrixDiagDyn MatrixDiagDyn; \
   typedef typename MyMatrix< PREC >::MatrixDynRow MatrixDynRow;

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
    typedef typename _CollisionSolverConfigName_::RigidBodyType RigidBodyType; \
    DEFINE_RIGIDBODY_CONFIG_TYPES_OF( _CollisionSolverConfigName_::RigidBodyType::RigidBodyConfigType )\

#define DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF( _DynamicsSystemConfigName_ ) \
    typedef typename _DynamicsSystemConfigName_::RigidBodyType RigidBodyType; \
    DEFINE_RIGIDBODY_CONFIG_TYPES_OF( _DynamicsSystemConfigName_::RigidBodyType::RigidBodyConfigType ) \


#define DEFINE_RIGIDBODY_CONFIG_TYPES_OF( _RigidBodyConfigName_ ) \
    typedef typename _RigidBodyConfigName_::LayoutConfigType LayoutConfigType; \
    typedef typename _RigidBodyConfigName_::RigidBodySolverDataType RigidBodySolverDataType; \
    DEFINE_LAYOUT_CONFIG_TYPES_OF( _RigidBodyConfigName_::LayoutConfigType ) \

/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argument typename â€œLayoutConfigâ€
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF( _LayoutConfigName_ ) \
   typedef typename _LayoutConfigName_::PREC PREC;             \
   static int const NDOFq = _LayoutConfigName_::LayoutType::NDOFq; \
   static int const NDOFu = _LayoutConfigName_::LayoutType::NDOFu; \
   static int const NDOFqObj = _LayoutConfigName_::LayoutType::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::LayoutType::NDOFuObj; \
   typedef typename _LayoutConfigName_::MatrixQU MatrixQU;     \
   typedef typename _LayoutConfigName_::MatrixQQ MatrixQQ;     \
   typedef typename _LayoutConfigName_::MatrixUU MatrixUU;     \
   typedef typename _LayoutConfigName_::MatrixUQ MatrixUQ;     \
   typedef typename _LayoutConfigName_::MatrixDiagUU MatrixDiagUU;     \
   typedef typename _LayoutConfigName_::MatrixDiagQQ MatrixDiagQQ;     \
   typedef typename _LayoutConfigName_::MatrixQDyn MatrixQDyn; \
   typedef typename _LayoutConfigName_::MatrixUDyn MatrixUDyn; \
   typedef typename _LayoutConfigName_::VectorQ VectorQ;       \
   typedef typename _LayoutConfigName_::VectorU VectorU;             \
   typedef typename _LayoutConfigName_::VectorQObj VectorQObj;       \
   typedef typename _LayoutConfigName_::VectorUObj VectorUObj;       \
   typedef typename _LayoutConfigName_::MatrixQObjUObj MatrixQObjUObj; \
   typedef typename _LayoutConfigName_::Matrix44 Matrix44; \
   typedef typename _LayoutConfigName_::Matrix33 Matrix33; \
   typedef typename _LayoutConfigName_::Matrix43 Matrix43; \
   typedef typename _LayoutConfigName_::Matrix34 Matrix34; \
   typedef typename _LayoutConfigName_::Vector3 Vector3;   \
   typedef typename _LayoutConfigName_::Vector4 Vector4;   \
   typedef typename _LayoutConfigName_::Quaternion Quaternion; \
   typedef typename _LayoutConfigName_::VectorDyn VectorDyn;   \
   typedef typename _LayoutConfigName_::MatrixDyn MatrixDyn;   \
   typedef typename _LayoutConfigName_::MatrixDiagDyn MatrixDiagDyn;   \
   typedef typename _LayoutConfigName_::MatrixDynRow MatrixDynRow;

/**
* @brief This macro is used to typedef all template arguments outside of a class for a specific LayoutConfig.
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF_OUTSIDE_TEMPLATE( _LayoutConfigName_ ) \
   typedef  _LayoutConfigName_::PREC PREC;             \
   static int const NDOFq = _LayoutConfigName_::LayoutType::NDOFq; \
   static int const NDOFu = _LayoutConfigName_::LayoutType::NDOFu; \
   static int const NDOFqObj = _LayoutConfigName_::LayoutType::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::LayoutType::NDOFuObj; \
   typedef  _LayoutConfigName_::MatrixQU MatrixQU;     \
   typedef  _LayoutConfigName_::MatrixQQ MatrixQQ;     \
   typedef  _LayoutConfigName_::MatrixUU MatrixUU;     \
   typedef  _LayoutConfigName_::MatrixUQ MatrixUQ;     \
   typedef  _LayoutConfigName_::MatrixDiagUU MatrixDiagUU;     \
   typedef  _LayoutConfigName_::MatrixDiagQQ MatrixDiagQQ;     \
   typedef  _LayoutConfigName_::MatrixQDyn MatrixQDyn; \
   typedef  _LayoutConfigName_::MatrixUDyn MatrixUDyn; \
   typedef  _LayoutConfigName_::VectorQ VectorQ;       \
   typedef  _LayoutConfigName_::VectorU VectorU;             \
   typedef  _LayoutConfigName_::VectorQObj VectorQObj;       \
   typedef  _LayoutConfigName_::VectorUObj VectorUObj;       \
   typedef  _LayoutConfigName_::MatrixQObjUObj MatrixQObjUObj; \
   typedef  _LayoutConfigName_::Matrix44 Matrix44; \
   typedef  _LayoutConfigName_::Matrix33 Matrix33; \
   typedef  _LayoutConfigName_::Matrix43 Matrix43; \
   typedef  _LayoutConfigName_::Matrix34 Matrix34; \
   typedef  _LayoutConfigName_::Vector3 Vector3;   \
   typedef  _LayoutConfigName_::Vector4 Vector4;   \
   typedef  _LayoutConfigName_::Quaternion Quaternion; \
   typedef  _LayoutConfigName_::VectorDyn VectorDyn;   \
   typedef  _LayoutConfigName_::MatrixDyn MatrixDyn;   \
   typedef  _LayoutConfigName_::MatrixDiagDyn MatrixDiagDyn;   \
   typedef  _LayoutConfigName_::MatrixDynRow MatrixDynRow;

/**
* @brief This is the format for the output of matrices.
*/
struct MyIOFormat{
  static Eigen::IOFormat Matlab;
};



typedef double MeshPREC;

/* @} */


#endif



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
template< typename _TLayoutConfig, typename _TSolverConfig, typename _TSystem >
struct Config{
   typedef _TLayoutConfig TLayoutConfig;
   typedef _TSolverConfig TSolverConfig;
   typedef _TSystem TSystem;
};

// ================================================================================================
/** @brief This
*	These are some small matrix definitions.
*/
template<typename TPREC>
struct MyMatrix{
   typedef TPREC PREC;
   //Static assigned Matrices
   typedef Eigen::Matrix<PREC, 4, 4> Matrix44;
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
   typedef TLayout Layout;

   // Dynamic or Static assigned Matrices
   typedef Eigen::Matrix<PREC, Layout::NDOFq, Layout::NDOFq>           MatrixQQ;
   typedef Eigen::Matrix<PREC, Layout::NDOFu, Layout::NDOFu>           MatrixUU;
   typedef Eigen::DiagonalMatrix<PREC, Layout::NDOFq>                  MatrixDiagQQ;
   typedef Eigen::DiagonalMatrix<PREC, Layout::NDOFu>                  MatrixDiagUU;
   
   typedef Eigen::Matrix<PREC, Layout::NDOFq, Layout::NDOFu>           MatrixQU;
   typedef Eigen::Matrix<PREC, Layout::NDOFu, Layout::NDOFq>           MatrixUQ;
   typedef Eigen::Matrix<PREC, Layout::NDOFu, Eigen::Dynamic >         MatrixUDyn;
   typedef Eigen::Matrix<PREC, Layout::NDOFq, Eigen::Dynamic >         MatrixQDyn;

   // Dynamic assigned Vectors
   typedef Eigen::Matrix<PREC, Layout::NDOFq, 1>                       VectorQ;
   typedef Eigen::Matrix<PREC, Layout::NDOFu, 1>                       VectorU;
   // Static assigned Vectors/Matrices
   typedef Eigen::Matrix<PREC, Layout::NDOFqObj, Layout::NDOFuObj>     MatrixQObjUObj;
   typedef Eigen::Matrix<PREC, Layout::NDOFqObj, 1>                    VectorQObj;
   typedef Eigen::Matrix<PREC, Layout::NDOFuObj, 1>                    VectorUObj;

   typedef Eigen::Matrix<PREC, Layout::NDOFFriction + 1, 1>            VectorPContact;
   typedef Eigen::Matrix<PREC, Layout::NDOFFriction, 1>                VectorPFriction;

   typedef typename MyMatrix< PREC >::Matrix44 Matrix44; 
   typedef typename MyMatrix< PREC >::Matrix33 Matrix33; 
   typedef typename MyMatrix< PREC >::Vector3 Vector3;   
   typedef typename MyMatrix< PREC >::Vector4 Vector4;   
   typedef typename MyMatrix< PREC >::Quaternion Quaternion; 
   typedef typename MyMatrix< PREC >::VectorDyn VectorDyn; 
   typedef typename MyMatrix< PREC >::MatrixDyn MatrixDyn; 
   typedef typename MyMatrix< PREC >::MatrixDiagDyn MatrixDiagDyn; 
   typedef typename MyMatrix< PREC >::MatrixDynRow MatrixDynRow; 

};

/**
* @brief A dynamic layout specialization.
*/
template <int nDOFqObj, int nDOFuObj, int nDOFFriction>
struct DynamicLayout {
   static int const NDOFq = -1;
   static int const NDOFu = -1;
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
   static int const NDOFFriction = nDOFFriction;

};

/**
* @brief A static layout specialization.
*/
template <int NObjects, int nDOFqObj, int nDOFuObj, int nDOFFriction>
struct StaticLayout {
   static int const NDOFqObj = nDOFqObj;
   static int const NDOFuObj = nDOFuObj;
   static int const NDOFq = NObjects*NDOFqObj;
   static int const NDOFu = NObjects*NDOFuObj;
   static int const NDOFFriction = nDOFFriction;
};

/**
* @brief A dynamic double layout specialization. For rigid bodies with NDOFq = 7 , NDOFu = 6 and NDOFFriction = 2.
*/
typedef LayoutConfig<
   double,
   DynamicLayout<7,6,2>
> DoubleDynamicLayout;


// ================================================================================================
/**
* @brief The solver config with TimeStepper, CollisionSolver and InclusionSolver
*/
template < typename _TTimeStepper,  typename _TCollisionSolver, typename _TInclusionSolver>
struct SolverConfig{
   typedef _TTimeStepper TTimeStepper;
   typedef _TCollisionSolver TCollisionSolver;
   typedef _TInclusionSolver TInclusionSolver;
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
class SpheresSystem;
template< typename TLayoutConfig > class StatePoolVisBackFront;
template< typename TLayoutConfig > class CollisionSolver;
template< typename TLayoutConfig > class ContactGraph;
template< typename TLayoutConfig > class DynamicsSystem;
template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver> class InclusionSolverNT;
template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TContactGraph > class InclusionSolverCO;
template< typename TLayoutConfig ,typename TDynamicsSystem, typename TCollisionSolver, typename TInclusionSolver,  typename TStatePool> class MoreauTimeStepper;

typedef DoubleDynamicLayout DoubleDynamicLayout;

typedef SolverConfig
   <
   MoreauTimeStepper<
      DoubleDynamicLayout,
      SpheresSystem,
      CollisionSolver<DoubleDynamicLayout>,
      InclusionSolverNT<DoubleDynamicLayout,SpheresSystem,CollisionSolver<DoubleDynamicLayout>>,
      StatePoolVisBackFront<DoubleDynamicLayout>
      >,
      CollisionSolver<DoubleDynamicLayout>,
      InclusionSolverNT<DoubleDynamicLayout,SpheresSystem,CollisionSolver<DoubleDynamicLayout> >
   > GeneralSolverConfigNotOrdered;

typedef SolverConfig
   <
      MoreauTimeStepper<
         DoubleDynamicLayout,
         DynamicsSystem<DoubleDynamicLayout>,
         CollisionSolver<DoubleDynamicLayout>,
         InclusionSolverCO<DoubleDynamicLayout,DynamicsSystem<DoubleDynamicLayout>,CollisionSolver<DoubleDynamicLayout>, ContactGraph<DoubleDynamicLayout> >,
         StatePoolVisBackFront<DoubleDynamicLayout>
      >,
      CollisionSolver<DoubleDynamicLayout>,
      InclusionSolverCO<
         DoubleDynamicLayout,
         DynamicsSystem<DoubleDynamicLayout>,
         CollisionSolver<DoubleDynamicLayout>, 
         ContactGraph<DoubleDynamicLayout> 
      >
   > GeneralSolverConfigOrdered;

typedef Config<DoubleDynamicLayout, GeneralSolverConfigOrdered, DynamicsSystem<DoubleDynamicLayout> > GeneralConfig;
/* @} */



// Macros ==================================================================================================================
/**
* @brief This macro is used to typedef all custom matrix types which have nothing to do with the system.
*/
#define DEFINE_MATRIX_TYPES \
   typedef typename MyMatrix< PREC >::Matrix44 Matrix44; \
   typedef typename MyMatrix< PREC >::Matrix33 Matrix33; \
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
   typedef typename _ConfigName_::TSystem TSystem; \
   typedef typename _ConfigName_::TLayoutConfig TLayoutConfig; \
   typedef typename _ConfigName_::TSolverConfig TSolverConfig; \
   typedef typename _ConfigName_::TSolverConfig::TTimeStepper TTimeStepper;                 \
   typedef typename _ConfigName_::TSolverConfig::TInclusionSolver TInclusionSolver;                 \
   typedef typename _ConfigName_::TSolverConfig::TCollisionSolver TCollisionSolver;                 \
   typedef typename _ConfigName_::TLayoutConfig::PREC PREC;             \
   static int const NDOFq = _ConfigName_::TLayoutConfig::Layout::NDOFq; \
   static int const NDOFu = _ConfigName_::TLayoutConfig::Layout::NDOFu; \
   static int const NDOFqObj = _ConfigName_::TLayoutConfig::Layout::NDOFqObj; \
   static int const NDOFuObj = _ConfigName_::TLayoutConfig::Layout::NDOFuObj; \
   static int const NDOFFriction = _ConfigName_::TLayoutConfig::Layout::NDOFFriction; \
   typedef typename _ConfigName_::TLayoutConfig::MatrixQU MatrixQU;     \
   typedef typename _ConfigName_::TLayoutConfig::MatrixQQ MatrixQQ;     \
   typedef typename _ConfigName_::TLayoutConfig::MatrixUU MatrixUU;     \
   typedef typename _ConfigName_::TLayoutConfig::MatrixUQ MatrixUQ;     \
   typedef typename _ConfigName_::TLayoutConfig::MatrixDiagUU MatrixDiagUU;     \
   typedef typename _ConfigName_::TLayoutConfig::MatrixDiagQQ MatrixDiagQQ;     \
   typedef typename _ConfigName_::TLayoutConfig::MatrixQDyn MatrixQDyn; \
   typedef typename _ConfigName_::TLayoutConfig::MatrixUDyn MatrixUDyn; \
   typedef typename _ConfigName_::TLayoutConfig::VectorQ VectorQ;       \
   typedef typename _ConfigName_::TLayoutConfig::VectorU VectorU;             \
   typedef typename _ConfigName_::TLayoutConfig::VectorQObj VectorQObj;       \
   typedef typename _ConfigName_::TLayoutConfig::VectorUObj VectorUObj;       \
   typedef typename _ConfigName_::TLayoutConfig::MatrixQObjUObj MatrixQObjUObj; \
   typedef typename _ConfigName_::TLayoutConfig::VectorPContact VectorPContact; \
   typedef typename _ConfigName_::TLayoutConfig::VectorPFriction VectorPFriction; \
   typedef typename _ConfigName_::TLayoutConfig::Matrix44 Matrix44; \
   typedef typename _ConfigName_::TLayoutConfig::Matrix33 Matrix33; \
   typedef typename _ConfigName_::TLayoutConfig::Vector3 Vector3;   \
   typedef typename _ConfigName_::TLayoutConfig::Vector4 Vector4;   \
   typedef typename _ConfigName_::TLayoutConfig::Quaternion Quaternion; \
   typedef typename _ConfigName_::TLayoutConfig::VectorDyn VectorDyn;   \
   typedef typename _ConfigName_::TLayoutConfig::MatrixDyn MatrixDyn;   \
   typedef typename _ConfigName_::TLayoutConfig::MatrixDiagDyn MatrixDiagDyn;   \
   typedef typename _ConfigName_::TLayoutConfig::MatrixDynRow MatrixDynRow; 

/**
* @brief This macro is used to typedef all template arguments in a class with e.g template argument typename â€œLayoutConfigâ€
*/
#define DEFINE_LAYOUT_CONFIG_TYPES_OF( _LayoutConfigName_ ) \
   typedef typename _LayoutConfigName_::PREC PREC;             \
   static int const NDOFq = _LayoutConfigName_::Layout::NDOFq; \
   static int const NDOFu = _LayoutConfigName_::Layout::NDOFu; \
   static int const NDOFqObj = _LayoutConfigName_::Layout::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::Layout::NDOFuObj; \
   static int const NDOFFriction = _LayoutConfigName_::Layout::NDOFFriction; \
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
   typedef typename _LayoutConfigName_::VectorPContact VectorPContact; \
   typedef typename _LayoutConfigName_::VectorPFriction VectorPFriction; \
    typedef typename _LayoutConfigName_::Matrix44 Matrix44; \
   typedef typename _LayoutConfigName_::Matrix33 Matrix33; \
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
   static int const NDOFq = _LayoutConfigName_::Layout::NDOFq; \
   static int const NDOFu = _LayoutConfigName_::Layout::NDOFu; \
   static int const NDOFqObj = _LayoutConfigName_::Layout::NDOFqObj; \
   static int const NDOFuObj = _LayoutConfigName_::Layout::NDOFuObj; \
   static int const NDOFFriction = _LayoutConfigName_::Layout::NDOFFriction; \
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
   typedef  _LayoutConfigName_::VectorPContact VectorPContact; \
   typedef  _LayoutConfigName_::VectorPFriction VectorPFriction; \
   typedef  _LayoutConfigName_::Matrix44 Matrix44; \
   typedef  _LayoutConfigName_::Matrix33 Matrix33; \
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



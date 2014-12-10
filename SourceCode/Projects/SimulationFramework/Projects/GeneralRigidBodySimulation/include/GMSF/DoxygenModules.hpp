


#ifndef GMSF_DoxygenModules_hpp
#define GMSF_DoxygenModules_hpp


/** @defgroup Common 
* These explaines the common classes which are located under the subfolder Common.
* Common classes can not really be classified, and are used as helper classes in the framework.
*/


/** @defgroup Dynamics 
* These explaines the classes which are related to the dynamics part of the framework. This includes all files
* under the subfolder Dynamics.
* The dynamics includes the whole mechanics part, solvers, collision, timestepper and many more helper classes to make up a efficient versatile 
* simulation framework.
*/

/**
* @ingroup Dynamics
* @defgroup Collision 
* These explaines the classes which are related to the collision part of the dynamics.
*/
/**
* @ingroup Dynamics
* @defgroup Contact 
* These explaines the classes which are related to a contact of the dynamics.
*/

/**
* @ingroup Dynamics
* @defgroup StatesAndBuffers States & Buffers 
* These explaines the classes which are related to the shared buffers and the states of the dynamics.
*/
/**
* @ingroup Dynamics
* @defgroup DynamicsGeneral General
* General stuff for the dynamics.
*/
/**
* @ingroup Dynamics
* @defgroup Inclusion 
* These are the classes and function corresponding to the inclusion problem.
*/



/** @defgroup App 
* These explaines the classes which are related to the application part of the framework. This includes all files
* under the subfolder App.
*/

/** @defgroup Singeltons 
* These explaines the classes which are related to a singelton in the framework. This includes all files
* under the subfolder System.
*/
/** 
* @ingroup Singeltons
* @defgroup Contexts 
* All the contexts in this framework.
*/

/** @defgroup States 
* These explaines the classes which are related to an app states in the framework. This includes all files
* under the subfolder States.
* An application can have several states, which are put in an application state queue. The way the user programs the states, they can be pushed
* and popped from the stack which then makes it possible to have for example several simulations of different systems in one application and the user
* can switch between them.
*/

/** @defgroup Systems 
* These explaines the classes which are related to a dynamics system in the framework. This includes all files
* under the subfolder System.
*/


#endif	

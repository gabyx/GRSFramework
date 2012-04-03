#ifndef DynamicsSystem_hpp
#define DynamicsSystem_hpp

#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>

#include "TypeDefs.hpp"
#include "LogDefines.hpp"

#include "RigidBody.hpp"
#include "DynamicsState.hpp"
#include "InclusionSolverSettings.hpp"
#include "CommonFunctions.hpp"

#include "TimeStepperSettings.hpp"


template<typename TLayoutConfig>
class DynamicsSystem
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   DEFINE_LAYOUT_CONFIG_TYPES_OF( TLayoutConfig )

	DynamicsSystem();
	~DynamicsSystem();

	// General related variables 
  double m_gravity;
  Vector3 m_gravityDir;

  std::vector< boost::shared_ptr<RigidBody<TLayoutConfig> > > m_SimBodies; // Simulated Objects
  std::vector< boost::shared_ptr<RigidBody<TLayoutConfig> > > m_Bodies;    // all not simulated objects

  void init(); // Only call if Timestepper has been created
  void initializeLog(Ogre::Log* pLog); 


  void init_MassMatrix(); // MassMatrix is const
  void init_MassMatrixInv(); // MassMatrix is const
  void init_const_hTerm(); // Initializes constant terms in h

	//Virtuals
  void doFirstHalfTimeStep( const DynamicsState<TLayoutConfig> * state_s,  DynamicsState<TLayoutConfig> * state_m, PREC timestep);
  void doSecondHalfTimeStep(const DynamicsState<TLayoutConfig> * state_m,  DynamicsState<TLayoutConfig> * state_e, PREC timestep);

  void getSettings(TimeStepperSettings<TLayoutConfig> &SettingsTimestepper, InclusionSolverSettings<TLayoutConfig> &SettingsInclusionSolver);
  void setSettings(const TimeStepperSettings<TLayoutConfig> &SettingsTimestepper, const InclusionSolverSettings<TLayoutConfig> &SettingsInclusionSolver);
 
	void reset();
	inline  void afterFirstTimeStep(const DynamicsState<TLayoutConfig> * state_s){};
	inline  void afterSecondTimeStep(const DynamicsState<TLayoutConfig> * state_s){};
	void doInputTimeStep(PREC T){};
  
  double m_CurrentStateEnergy;

protected:

  TimeStepperSettings<TLayoutConfig> m_SettingsTimestepper;
  InclusionSolverSettings<TLayoutConfig> m_SettingsInclusionSolver;

   //Inits
	void initializeGlobalParameters();
  
  //Function
  //This is a minimal update of F, no checking if constant values are correct
  void updateFMatrix(const VectorQObj & q, MatrixQObjUObj & F_i);
  
 
  // Log
	Ogre::Log*	m_pSolverLog;
	std::stringstream logstream;

};




#include "VectorToSkewMatrix.hpp" 


template<typename TLayoutConfig>
DynamicsSystem<TLayoutConfig>::DynamicsSystem()
{

  // Set standart values for settings
  m_SettingsTimestepper.m_deltaT = 0.001;
  m_SettingsTimestepper.m_endTime = 10;
  m_SettingsTimestepper.m_stateReferenceFile = boost::filesystem::path();
  m_SettingsTimestepper.m_eSimulateFromReference = TimeStepperSettings<TLayoutConfig>::NONE;

  m_SettingsInclusionSolver.m_deltaT = 0.001;
  m_SettingsInclusionSolver.m_alphaJORProx = 0.5;
  m_SettingsInclusionSolver.m_alphaSORProx = 1.2;
  m_SettingsInclusionSolver.m_MaxIter = 5000;
  m_SettingsInclusionSolver.m_AbsTol = 1E-7;
  m_SettingsInclusionSolver.m_RelTol = 1E-7;
  m_SettingsInclusionSolver.m_eMethod = InclusionSolverSettings<TLayoutConfig>::SOR;
  m_SettingsInclusionSolver.m_bUseGPU = false;
  m_SettingsInclusionSolver.m_UseGPUDeviceId = 0;
  m_SettingsInclusionSolver.m_bIsFiniteCheck = false;
};
template<typename TLayoutConfig>
DynamicsSystem<TLayoutConfig>::~DynamicsSystem()
{
  DECONSTRUCTOR_MESSAGE
};

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::init()
{
  initializeGlobalParameters();
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::initializeGlobalParameters()
{
  //m_mass = 0.050;
  m_gravity = 9.81;
  m_gravityDir = Vector3(0,0,-1);
 /* m_ThetaS_A = 2.0/5.0 * m_mass * (m_R*m_R);
  m_ThetaS_B = 2.0/5.0 * m_mass * (m_R*m_R);
  m_ThetaS_C = 2.0/5.0 * m_mass * (m_R*m_R);*/
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::getSettings(TimeStepperSettings<TLayoutConfig> &SettingsTimestepper, InclusionSolverSettings<TLayoutConfig> &SettingsInclusionSolver)
{
  SettingsTimestepper = m_SettingsTimestepper;
  SettingsInclusionSolver = m_SettingsInclusionSolver;
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::setSettings(const TimeStepperSettings<TLayoutConfig> &SettingsTimestepper, const InclusionSolverSettings<TLayoutConfig> &SettingsInclusionSolver)
{
  m_SettingsTimestepper = SettingsTimestepper;
  m_SettingsInclusionSolver = SettingsInclusionSolver;
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::initializeLog(Ogre::Log* pLog)
{
  m_pSolverLog = pLog;
  ASSERTMSG(m_pSolverLog != NULL, "Ogre::Log: NULL!");
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::reset(){
  
}
template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::doFirstHalfTimeStep(const DynamicsState<TLayoutConfig> * state_s,  
                                       DynamicsState<TLayoutConfig> * state_m, 
                                       PREC timestep){

  static MatrixQObjUObj F_i = MatrixQObjUObj::Identity();

  // Do timestep for every object

  state_m->m_t =  state_s->m_t + timestep;
  
#if CoutLevelSolver>0
  CLEARLOG;
  logstream <<"m_t_s= "  <<state_s->m_t<<endl; 
  logstream <<"m_t_m= "  <<state_m->m_t<<endl; 
  LOG(m_pSolverLog);
#endif

  for(int i=0;i< state_s->m_SimBodyStates.size();i++){
    // Set F for this object:
    updateFMatrix(state_s->m_SimBodyStates[i].m_q, F_i);
    // Timestep for the object
    state_m->m_SimBodyStates[i].m_q = state_s->m_SimBodyStates[i].m_q    +   timestep * F_i * state_s->m_SimBodyStates[i].m_u;

    // Update objects state to the actual state
    m_SimBodies[i]->m_r_S  = state_m->m_SimBodyStates[i].m_q.head<3>();
    m_SimBodies[i]->m_q_KI = state_m->m_SimBodyStates[i].m_q.tail<4>();

    // Update Transformation A_IK
    setRotFromQuaternion<>(state_m->m_SimBodyStates[i].m_q.tail<4>(), m_SimBodies[i]->m_A_IK);


     // Add in to h-Term ==========
     m_SimBodies[i]->m_h_term = m_SimBodies[i]->m_h_term_const;
        // Term omega x Theta * omega = 0, because theta is diagonal
     // =========================

    // Add in to Mass Matrix
       // Mass Matrix is Constant!
    // =================

#if CoutLevelSolver>2
    CLEARLOG;
    logstream  <<i<<" : m_q_M= "  <<state_m->m_SimBodyStates[i].m_q.transpose()<<endl;
    logstream  <<i<<" : m_u_S= "  <<state_s->m_SimBodyStates[i].m_u.transpose()<<endl; 
    LOG(m_pSolverLog);
#endif
  }
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::doSecondHalfTimeStep(const DynamicsState<TLayoutConfig> * state_m,  DynamicsState<TLayoutConfig> * state_e, PREC timestep){

  static MatrixQObjUObj F_i = MatrixQObjUObj::Identity();
  // Do timestep for every object


  state_e->m_t =  state_m->m_t + timestep;
#if CoutLevelSolver>0
  CLEARLOG;
  logstream <<"m_t_e= "  <<state_e->m_t<<endl; 
  LOG(m_pSolverLog);
#endif
  m_CurrentStateEnergy = 0;
  for(int i=0;i< state_m->m_SimBodyStates.size();i++){
    // Set F for this object:
    updateFMatrix(state_m->m_SimBodyStates[i].m_q, F_i);
    // Timestep for the object
    state_e->m_SimBodyStates[i].m_q = state_m->m_SimBodyStates[i].m_q    +   timestep * F_i * state_e->m_SimBodyStates[i].m_u;
    

    //Normalize Quaternion
    state_e->m_SimBodyStates[i].m_q.tail(4).normalize();

    
#if OUTPUT_SYSTEMDATA_FILE == 1
    // Calculate Energy
    m_CurrentStateEnergy += 0.5* state_e->m_SimBodyStates[i].m_u.transpose() * m_SimBodies[i]->m_MassMatrix_diag.asDiagonal() * state_e->m_SimBodyStates[i].m_u;
    m_CurrentStateEnergy -= + m_SimBodies[i]->m_mass * state_e->m_SimBodyStates[i].m_q.head<3>().transpose() * m_gravity*m_gravityDir ;
#endif

#if CoutLevelSolver>2
    CLEARLOG;
    logstream  <<i<<" : m_q_E= "  <<state_e->m_SimBodyStates[i].m_q.transpose()<<endl;
    logstream  <<i<<" : m_u_E= "  <<state_e->m_SimBodyStates[i].m_u.transpose()<<endl; 
    LOG(m_pSolverLog);
#endif
  }

}
template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::updateFMatrix(const VectorQObj & q, MatrixQObjUObj & F_i)
{
  static MyMatrix<PREC>::Matrix33 a_tilde = MyMatrix<PREC>::Matrix33::Zero();

  F_i.block<1,3>(3,3) = -0.5 * q.tail<3>();
  updateSkewSymmetricMatrix<>(q.tail<3>(), a_tilde );
  F_i.block<3,3>(4,3) = 0.5 * ( Matrix<double,3,3>::Identity() * q(3) + a_tilde );
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::init_MassMatrix(){
  // iterate over all objects and assemble matrix M
  for(int i=0; i < m_SimBodies.size();i++){
     m_SimBodies[i]->m_MassMatrix_diag.head<3>().setConstant(m_SimBodies[i]->m_mass);
     m_SimBodies[i]->m_MassMatrix_diag.tail<3>() = m_SimBodies[i]->m_K_Theta_S;
  }
}

template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::init_MassMatrixInv(){
  // iterate over all objects and assemble matrix M inverse
  for(int i=0; i < m_SimBodies.size();i++){
     m_SimBodies[i]->m_MassMatrixInv_diag = m_SimBodies[i]->m_MassMatrix_diag.array().inverse().matrix();
  }
}
template<typename TLayoutConfig>
void DynamicsSystem<TLayoutConfig>::init_const_hTerm()
{
   // Fill in constant terms of h-Term
   for(int i=0; i < m_SimBodies.size();i++){
        m_SimBodies[i]->m_h_term_const.head<3>() =  m_SimBodies[i]->m_mass * m_gravity * m_gravityDir;
   }
}


#endif 
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

#include "SimpleLogger.hpp"


template<typename TDynamicsSystemConfig>
class DynamicsSystem
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   typedef TDynamicsSystemConfig DynamicsSystemConfig;
   DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(TDynamicsSystemConfig)

	DynamicsSystem();
	~DynamicsSystem();

	// General related variables
  double m_gravity;
  Vector3 m_gravityDir;

  std::vector< boost::shared_ptr<RigidBodyType > > m_SimBodies; // Simulated Objects
  std::vector< boost::shared_ptr<RigidBodyType > > m_Bodies;    // all not simulated objects

  void init(); // Only call if Timestepper has been created
  void initializeLog(Logging::Log* pLog);


  void init_MassMatrix(); // MassMatrix is const
  void init_MassMatrixInv(); // MassMatrix is const
  void init_const_hTerm(); // Initializes constant terms in h

	//Virtuals
  void doFirstHalfTimeStep( const DynamicsState<LayoutConfigType> * state_s,  DynamicsState<LayoutConfigType> * state_m, PREC timestep);
  void doSecondHalfTimeStep(const DynamicsState<LayoutConfigType> * state_m,  DynamicsState<LayoutConfigType> * state_e, PREC timestep);

  void getSettings(TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver);
  void setSettings(const TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, const InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver);

	void reset();
	inline  void afterFirstTimeStep(const DynamicsState<LayoutConfigType> * state_s){};
	inline  void afterSecondTimeStep(const DynamicsState<LayoutConfigType> * state_s){};
	void doInputTimeStep(PREC T){};

  double m_CurrentStateEnergy;

protected:

  TimeStepperSettings<LayoutConfigType> m_SettingsTimestepper;
  InclusionSolverSettings<LayoutConfigType> m_SettingsInclusionSolver;

   //Inits
	void initializeGlobalParameters();

  //Function
  //This is a minimal update of F, no checking if constant values are correct
  void updateFMatrix(const VectorQObj & q, MatrixQObjUObj & F_i);


  // Log
	Logging::Log*	m_pSolverLog;
	std::stringstream logstream;

};




#include "VectorToSkewMatrix.hpp"


template<typename TDynamicsSystemConfig>
DynamicsSystem<TDynamicsSystemConfig>::DynamicsSystem()
{


};
template<typename TDynamicsSystemConfig>
DynamicsSystem<TDynamicsSystemConfig>::~DynamicsSystem()
{
  DECONSTRUCTOR_MESSAGE
};

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::init()
{
  initializeGlobalParameters();
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::initializeGlobalParameters()
{
  //m_mass = 0.050;
  m_gravity = 9.81;
  m_gravityDir = Vector3(0,0,-1);
 /* m_ThetaS_A = 2.0/5.0 * m_mass * (m_R*m_R);
  m_ThetaS_B = 2.0/5.0 * m_mass * (m_R*m_R);
  m_ThetaS_C = 2.0/5.0 * m_mass * (m_R*m_R);*/
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::getSettings(TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver)
{
  SettingsTimestepper = m_SettingsTimestepper;
  SettingsInclusionSolver = m_SettingsInclusionSolver;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::setSettings(const TimeStepperSettings<LayoutConfigType> &SettingsTimestepper, const InclusionSolverSettings<LayoutConfigType> &SettingsInclusionSolver)
{
  m_SettingsTimestepper = SettingsTimestepper;
  m_SettingsInclusionSolver = SettingsInclusionSolver;
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::initializeLog(Logging::Log* pLog)
{
  m_pSolverLog = pLog;
  ASSERTMSG(m_pSolverLog != NULL, "Logging::Log: NULL!");
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::reset(){

}
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::doFirstHalfTimeStep(const DynamicsState<LayoutConfigType> * state_s,
                                       DynamicsState<LayoutConfigType> * state_m,
                                       PREC timestep){
  using namespace std;
  static MatrixQObjUObj F_i = MatrixQObjUObj::Identity();

  // Do timestep for every object

  state_m->m_t =  state_s->m_t + timestep;

#if CoutLevelSolver>0
  LOG(m_pSolverLog, << "m_t_s= "  <<state_s->m_t<<endl
                    << "m_t_m= "  <<state_m->m_t<<endl;)
#endif

  for(int i=0;i< state_s->m_SimBodyStates.size();i++){
    // Set F for this object:
    updateFMatrix(state_s->m_SimBodyStates[i].m_q, F_i);
    // Timestep for the object
    state_m->m_SimBodyStates[i].m_q = state_s->m_SimBodyStates[i].m_q    +   timestep * F_i * state_s->m_SimBodyStates[i].m_u;

    // Update objects state to the actual state
    m_SimBodies[i]->m_r_S  = state_m->m_SimBodyStates[i].m_q.template head<3>();
    m_SimBodies[i]->m_q_KI = state_m->m_SimBodyStates[i].m_q.template tail<4>();

    // Update Transformation A_IK
    setRotFromQuaternion<>(state_m->m_SimBodyStates[i].m_q.template tail<4>(), m_SimBodies[i]->m_A_IK);


     // Add in to h-Term ==========
     m_SimBodies[i]->m_h_term = m_SimBodies[i]->m_h_term_const;
        // Term omega x Theta * omega = 0, because theta is diagonal
     // =========================

    // Add in to Mass Matrix
       // Mass Matrix is Constant!
    // =================

#if CoutLevelSolver>2
    LOG(m_pSolverLog,   << i <<" : m_q_M= "  <<state_m->m_SimBodyStates[i].m_q.transpose()<<std::endl
                        << i <<" : m_u_S= "  <<state_s->m_SimBodyStates[i].m_u.transpose()<<std::endl;)
#endif
  }
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::doSecondHalfTimeStep(const DynamicsState<LayoutConfigType> * state_m,  DynamicsState<LayoutConfigType> * state_e, PREC timestep){

    using namespace std;

  static MatrixQObjUObj F_i = MatrixQObjUObj::Identity();
  // Do timestep for every object


  state_e->m_t =  state_m->m_t + timestep;
#if CoutLevelSolver>0
  LOG(m_pSolverLog, << "m_t_e= "  <<state_e->m_t<<endl)
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
    m_CurrentStateEnergy -= + m_SimBodies[i]->m_mass * state_e->m_SimBodyStates[i].m_q.template head<3>().transpose() * m_gravity*m_gravityDir ;
#endif

#if CoutLevelSolver>2
    LOG(m_pSolverLog,   << i<<" : m_q_E= "  <<state_e->m_SimBodyStates[i].m_q.transpose()<<endl
                        << i<<" : m_u_E= "  <<state_e->m_SimBodyStates[i].m_u.transpose()<<endl)
#endif
  }

}
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::updateFMatrix(const VectorQObj & q, MatrixQObjUObj & F_i)
{
  static Matrix33 a_tilde = Matrix33::Zero();

  F_i.template block<1,3>(3,3) = -0.5 * q.template tail<3>();
  updateSkewSymmetricMatrix<>(q.template tail<3>(), a_tilde );
  F_i.template block<3,3>(4,3) = 0.5 * ( Matrix33::Identity() * q(3) + a_tilde );
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::init_MassMatrix(){
  // iterate over all objects and assemble matrix M
  for(int i=0; i < m_SimBodies.size();i++){
     m_SimBodies[i]->m_MassMatrix_diag.template head<3>().setConstant(m_SimBodies[i]->m_mass);
     m_SimBodies[i]->m_MassMatrix_diag.template tail<3>() = m_SimBodies[i]->m_K_Theta_S;
  }
}

template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::init_MassMatrixInv(){
  // iterate over all objects and assemble matrix M inverse
  for(int i=0; i < m_SimBodies.size();i++){
     m_SimBodies[i]->m_MassMatrixInv_diag = m_SimBodies[i]->m_MassMatrix_diag.array().inverse().matrix();
  }
}
template<typename TDynamicsSystemConfig>
void DynamicsSystem<TDynamicsSystemConfig>::init_const_hTerm()
{
   // Fill in constant terms of h-Term
   for(int i=0; i < m_SimBodies.size();i++){
        m_SimBodies[i]->m_h_term_const.template head<3>() =  m_SimBodies[i]->m_mass * m_gravity * m_gravityDir;
   }
}


#endif

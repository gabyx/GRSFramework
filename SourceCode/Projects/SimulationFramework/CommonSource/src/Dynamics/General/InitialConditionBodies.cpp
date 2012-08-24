
#include "InitialConditionBodies.hpp"

#include "RigidBody.hpp"

namespace InitialConditionBodies {

void setupBodiesLinear(
    DynamicsState & init_state,
    MyLayoutConfigType::Vector3 pos,
    MyLayoutConfigType::Vector3 dir,
    double dist, bool jitter, double delta, unsigned int seed) {

    DEFINE_LAYOUT_CONFIG_TYPES_OF_OUTSIDE_TEMPLATE(MyLayoutConfigType)


    dir.normalize();
    Vector3 jitter_vec, random_vec;
    jitter_vec.setZero();

    // Set only m_q, m_u is zero in constructor!
    double d = 2; //spread around origin with 0.5m
    for(unsigned int i=0; i< init_state.m_nSimBodies; i++) {
        init_state.m_SimBodyStates[i].m_q.tail<4>() = Quaternion(1,0,0,0);

        typedef boost::mt19937  RNG;
        static  RNG generator(seed);
        static  boost::uniform_real<PREC> uniform(-1.0,1.0);
        static  boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

        if(jitter) {
            random_vec = Vector3(randomNumber(),randomNumber(),randomNumber()); // No uniform distribution!, but does not matter
            random_vec.normalize();
            random_vec = random_vec.cross(dir);
            random_vec.normalize();
            jitter_vec = random_vec * delta;
        }

        init_state.m_SimBodyStates[i].m_q.head<3>() = pos + dir*dist*i + jitter_vec;
    }
}

void setupBodiesGrid(DynamicsState & init_state,
                     unsigned int gDim_x,
                     unsigned int gDim_y,
                     double d,
                     MyLayoutConfigType::Vector3 vec_trans,
                     bool jitter,
                     double delta,
                     unsigned int seed) {
    DEFINE_LAYOUT_CONFIG_TYPES_OF_OUTSIDE_TEMPLATE(MyLayoutConfigType)

    Vector3 jitter_vec;
    jitter_vec.setZero();

    for(unsigned int i=0; i< init_state.m_nSimBodies; i++) {
        init_state.m_SimBodyStates[i].m_q.tail<4>() = Quaternion(1,0,0,0);
        int index_z = (i /(gDim_x*gDim_y));
        int index_y = (i - index_z*(gDim_x*gDim_y)) / gDim_y;
        int index_x = (i - index_z*(gDim_x*gDim_y)- index_y*gDim_y);
        //cout << index_x<<","<< index_y<<","<< index_z<<endl;
        double x = -d/2 + d/(double)(init_state.m_nSimBodies) * i;

        typedef boost::mt19937  RNG;
        static RNG generator(seed);
        static boost::uniform_real<PREC> uniform(-1.0,1.0);
        static boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

        if(jitter) {
            jitter_vec = Vector3(randomNumber(),randomNumber(),randomNumber()) * delta; // No uniform distribution!, but does not matter
        }

        init_state.m_SimBodyStates[i].m_q.head<3>() = Vector3(index_x * d - 0.5*(gDim_x-1)*d, index_y*d - 0.5*(gDim_y-1)*d , index_z*d) + vec_trans + jitter_vec;
    }

}


bool setupBodiesFromFile(DynamicsState & init_state, boost::filesystem::path file_path) {

    MultiBodySimFile simFile;

    if(simFile.openSimFileRead(file_path,init_state.m_nSimBodies)) {
        ;
        simFile >> init_state ;
        simFile.closeSimFile();
        return true;
    }

    return false;
}


void setupBodyPositionAxisAngle(RigidBodyState & rigibodyState, const MyLayoutConfigType::Vector3 & pos, MyLayoutConfigType::Vector3 & axis, MyLayoutConfigType::PREC angleRadian) {

    rigibodyState.m_q.head<3>() = pos;
    setQuaternion(rigibodyState.m_q.tail<4>(),axis,angleRadian);
}



void applyDynamicsStateToBodies(const DynamicsState & state, std::vector<boost::shared_ptr<MyRigidBodyType > > & bodies) {
    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );

    for(int i=0; i < bodies.size(); i++) {
        bodies[i]->m_r_S = state.m_SimBodyStates[i].m_q.head<3>();
        bodies[i]->m_q_KI= state.m_SimBodyStates[i].m_q.tail<4>();

        if(bodies[i]->m_eState == MyRigidBodyType::SIMULATED) {
            bodies[i]->m_pSolverData->m_uBuffer.m_Back = state.m_SimBodyStates[i].m_u;
            bodies[i]->m_pSolverData->m_t = state.m_t;
        }

        bodies[i]->m_A_IK= getRotFromQuaternion<>(bodies[i]->m_q_KI);
    }
}


void applyBodiesToDynamicsState(const std::vector<boost::shared_ptr<MyRigidBodyType > > & bodies, DynamicsState & state ) {
    ASSERTMSG(state.m_nSimBodies == bodies.size(), "Wrong Size" );

    std::vector<boost::shared_ptr<MyRigidBodyType > >::const_iterator bodyIt;
    DynamicsState::RigidBodyStateListType::iterator stateBodyIt = state.m_SimBodyStates.begin();
    MyRigidBodyType * pBody;

    for(bodyIt = bodies.begin(); bodyIt != bodies.end() ; bodyIt++) {
        pBody = (*bodyIt).get();

        stateBodyIt->m_q.head<3>() = pBody->m_r_S;
        stateBodyIt->m_q.tail<4>() = pBody->m_q_KI;

        if(pBody->m_pSolverData) {
            stateBodyIt->m_u = pBody->m_pSolverData->m_uBuffer.m_Back;
        }

        stateBodyIt++;
    }
}

}

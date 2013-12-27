
#include "TypeDefs.hpp"
#include "InitialConditionBodies.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "AssertionDebug.hpp"

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

void InitialConditionBodies::setupPositionBodiesLinear(
    DynamicsState & init_state,
    typename DynamicsState::Vector3 pos,
    typename DynamicsState::Vector3 dir,
    double dist, bool jitter, double delta, unsigned int seed) {

    DEFINE_LAYOUT_CONFIG_TYPES


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

void InitialConditionBodies::setupPositionBodiesGrid(DynamicsState & init_state,
                     unsigned int gDim_x,
                     unsigned int gDim_y,
                     double d,
                     typename DynamicsState::Vector3 vec_trans,
                     bool jitter,
                     double delta,
                     unsigned int seed) {

    DEFINE_LAYOUT_CONFIG_TYPES

    Vector3 jitter_vec;
    jitter_vec.setZero();

    for(unsigned int i=0; i< init_state.m_nSimBodies; i++) {
        init_state.m_SimBodyStates[i].m_q.tail<4>() = Quaternion(1,0,0,0);
        int index_z = (i /(gDim_x*gDim_y));
        int index_y = (i - index_z*(gDim_x*gDim_y)) / gDim_x;
        int index_x = (i - index_z*(gDim_x*gDim_y)- index_y*gDim_x);

        typedef boost::mt19937  RNG;
        static RNG generator(seed);
        static boost::uniform_real<PREC> uniform(-1.0,1.0);
        static boost::variate_generator< boost::mt19937 & , boost::uniform_real<PREC> > randomNumber(generator, uniform);

        if(jitter) {
            jitter_vec = Vector3(randomNumber(),randomNumber(),randomNumber()) * delta; // No uniform distribution!, but does not matter
        }

        init_state.m_SimBodyStates[i].m_q. head<3>() = Vector3(index_x * d - 0.5*(gDim_x-1)*d, index_y*d - 0.5*(gDim_y-1)*d , index_z*d) + vec_trans + jitter_vec;
    }

}

bool InitialConditionBodies::setupPositionBodiesFromFile(DynamicsState & init_state, boost::filesystem::path file_path) {

    MultiBodySimFile simFile( DynamicsState::LayoutConfigType::LayoutType::NDOFqBody,
                              DynamicsState::LayoutConfigType::LayoutType::NDOFuBody);

    if(simFile.openRead(file_path,init_state.m_nSimBodies)) {
        simFile >> init_state ;
        simFile.close();
        return true;
    }

    return false;
}


void InitialConditionBodies::setupPositionBodyPosAxisAngle(RigidBodyState & rigibodyState,
                                   const typename RigidBodyState::Vector3 & pos,
                                   typename RigidBodyState::Vector3 & axis,
                                   typename RigidBodyState::PREC angleRadian) {

    rigibodyState.m_q.head<3>() = pos;
    setQuaternion(rigibodyState.m_q.tail<4>(),axis,angleRadian);
}


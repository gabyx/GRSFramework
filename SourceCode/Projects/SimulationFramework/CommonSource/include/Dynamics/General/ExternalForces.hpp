#ifndef ExternalForces_hpp
#define ExternalForces_hpp

#include <functional>
#include <cmath>

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

template<typename TDynamicsSystem>
class SpatialUniformTimeRandomForceField{
    public:
        typedef TDynamicsSystem DynamicsSystemType;
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemType::DynamicsSystemConfig);

        SpatialUniformTimeRandomForceField(unsigned int seed, PREC boostTime, PREC pauseTime, PREC amplitude):
            m_boostTime(boostTime),m_pauseTime(pauseTime),m_seed(seed), m_amplitude(amplitude)
            {
                reset();
            }

        ~SpatialUniformTimeRandomForceField(){
            delete m_randomG;
        }
        void operator()(RigidBodyType * body){
            addForce(body);
        }
        void addForce(RigidBodyType * body){



            PREC t = std::fmod(body->m_pSolverData->m_t, (m_boostTime+m_pauseTime)) ;
            if(t < m_boostTime){
                Vector3 r = Vector3(body->m_r_S(0)+m_offsetX,body->m_r_S(1)+m_offsetY, 0.5+ body->m_r_S(2) + m_offsetZ)  ;
                r.normalize();
                body->m_h_term.template head<3>() += r*m_amplitude;
            }else{
                //set location of the spherical boost new
                m_offsetX = (*m_randomG)();
                m_offsetY = (*m_randomG)();
                m_offsetZ = (*m_randomG)();
                // apply no force =)
            }
        }

        void reset(){
            if(m_randomG){
                delete m_randomG;
            }
            typedef boost::mt19937  RNG;
            RNG generator(m_seed);
            boost::uniform_int<unsigned int> uniform(-1,1);
            m_randomG  = new boost::variate_generator< boost::mt19937 & , boost::uniform_int<unsigned int> > (generator, uniform);

            m_offsetX=0;
            m_offsetY=0;
            m_offsetZ=0;
        }

    private:
        boost::variate_generator< boost::mt19937 & , boost::uniform_int<unsigned int> > * m_randomG;
        PREC m_boostTime, m_pauseTime, m_amplitude, m_offsetX, m_offsetY, m_offsetZ ;
        unsigned int m_seed ;
};


template<typename TDynamicsSystem>
class ExternalForceList{
    public:
        typedef TDynamicsSystem DynamicsSystemType;
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemType::DynamicsSystemConfig);

        typedef typename std::vector< std::function<void (RigidBodyType *)> >::iterator iterator;

        // Wants a new pointer, it takes care of deleting the objects!
        template<typename T>
        void addExternalForceCalculation(T * extForce){
            m_deleterList.push_back( ExternalForceList::DeleteFunctor<T>(extForce) );
            m_calculationList.push_back( *extForce  );
        }

        ~ExternalForceList(){
            for (auto it = m_deleterList.begin(); it != m_deleterList.end(); it++){
                (*it)(); // delete all objects
            }
        }

        iterator begin(){return m_calculationList.begin();}
        iterator end(){return m_calculationList.end();}

    private:
        typedef std::vector< std::function<void (RigidBodyType *)> > CalcListType;


        std::vector< std::function<void (RigidBodyType *)> > m_calculationList;
        std::vector< std::function<void (void)> > m_deleterList;

        template<typename T>
        class DeleteFunctor {
            public:
                DeleteFunctor( T * p ):_p(p){};
                void operator()(void){delete _p;};
            private:
                T * _p;
        };

};

#endif

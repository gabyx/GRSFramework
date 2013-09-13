#ifndef ExternalForces_hpp
#define ExternalForces_hpp

#include <functional>
#include <cmath>

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

#include "AABB.hpp"

template<typename TDynamicsSystem>
class SpatialSphericalTimeRandomForceField{
    public:
        typedef TDynamicsSystem DynamicsSystemType;
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES_OF(DynamicsSystemType::DynamicsSystemConfig);

        SpatialSphericalTimeRandomForceField(unsigned int seed,
                                           PREC boostTime,
                                           PREC pauseTime,
                                           PREC amplitude,
                                           AABB<LayoutConfigType> randomBox,
                                           bool randomOn
                                           ):
            m_boostTime(boostTime),m_pauseTime(pauseTime),m_seed(seed), m_amplitude(amplitude), m_randomBox(randomBox), m_randomOn(randomOn)
        {
            m_randomG = NULL;
            reset();
        }

        ~SpatialSphericalTimeRandomForceField(){
            delete m_randomG;
        }
        void calculate(RigidBodyType * body){
            ASSERTMSG(body->m_pSolverData, "Solverdata not present!")
            if(m_ts > m_pauseTime){
                Vector3 r = body->m_r_S - m_offset ;
                r.normalize();
                body->m_h_term.template head<3>() += r*m_amplitude;
            }
        }

        void setTime(PREC time){

            m_t = time;
            m_ts = std::fmod(time, (m_boostTime+m_pauseTime)) ;

            if(m_randomOn){
                if(m_newPos == false && m_ts<m_pauseTime){
                    Vector3 ex = m_randomBox.extent();
                    m_offset = m_randomBox.m_minPoint + Vector3( ex(0)* (*m_randomG)(),  ex(1) * (*m_randomG)() , ex(2) * (*m_randomG)() );
                    m_newPos = true;
                }else{
                    m_newPos = false;
                }
            }
        }

        void reset(){
            if(m_randomG){
                delete m_randomG;
            }

            boost::mt19937  generator(m_seed);
            boost::uniform_real<PREC> uniform(0,1);
            m_randomG  = new boost::variate_generator< boost::mt19937 , boost::uniform_real<PREC> >(generator, uniform);

            m_t = 0;
            m_ts = 0;
            m_newPos = true;
            m_offset = m_randomBox.m_minPoint;
        }



    private:

        SpatialSphericalTimeRandomForceField(const SpatialSphericalTimeRandomForceField &);

        boost::variate_generator< boost::mt19937 , boost::uniform_real<PREC> > * m_randomG;
        PREC m_boostTime, m_pauseTime, m_amplitude, m_t, m_ts ;
        Vector3 m_offset;
        bool m_newPos, m_randomOn;
        AABB<LayoutConfigType> m_randomBox;
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
            m_resetList.push_back( std::bind( &T::reset, extForce ) );
            m_calculationList.push_back( std::bind(&T::calculate, extForce, std::placeholders::_1 ) );
            m_setTimeList.push_back( std::bind(&T::setTime, extForce, std::placeholders::_1 ) );
        }

        ~ExternalForceList(){
            //std::cout << "DELETE EXTERNAL FORCES" << std::endl;
            for (auto it = m_deleterList.begin(); it != m_deleterList.end(); it++){
                (*it)(); // delete all objects
            }
        }

        void reset(){
            //std::cout << "RESET EXTERNAL FORCES" << std::endl;
            for(auto it = m_resetList.begin(); it != m_resetList.end(); it++){
                (*it)(); // reseter list
            }
        }

        void setTime(PREC time){
            for(auto it = m_setTimeList.begin(); it != m_setTimeList.end(); it++){
                (*it)(time); // reseter list
            }
        }

        void calculate(RigidBodyType * body){
            for(auto it = m_calculationList.begin(); it != m_calculationList.end();it++){
                (*it)(body); // Apply calculation function!
            }
        }

        iterator begin(){return m_calculationList.begin();}
        iterator end(){return m_calculationList.end();}

    private:
        typedef std::vector< std::function<void (RigidBodyType *)> > CalcListType;


        CalcListType m_calculationList;
        std::vector< std::function<void (void)> > m_resetList;
        std::vector< std::function<void (PREC)> > m_setTimeList;
        std::vector< std::function<void (void)> > m_deleterList;

        // Must be copy constructable!
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

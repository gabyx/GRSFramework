#ifndef ExternalForces_hpp
#define ExternalForces_hpp



#include <functional>
#include <cmath>

#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>

#include "TypeDefs.hpp"

#include "AABB.hpp"


class GravityForceField{
    public:

        DEFINE_LAYOUT_CONFIG_TYPES

        static const bool m_addCalculate = true;
        static const bool m_addSetTime = false;
        static const bool m_addReset = false;

        GravityForceField(Vector3 gravityAccel): m_gravityAccel(gravityAccel){}
        ~GravityForceField(){}

        template<typename TRigidBody>
        void calculate(TRigidBody * body){
            body->m_h_term.template head<3>() += body->m_mass * m_gravityAccel;
        }

        void setTime(PREC time){};
        void reset(){};
    private:
        Vector3 m_gravityAccel;
};




class SpatialSphericalTimeRandomForceField{
    public:

        DEFINE_LAYOUT_CONFIG_TYPES

        static const bool m_addCalculate = true; // Decide if we add a calculate(...) as a function pointer in the ExternalForceList.
        static const bool m_addSetTime = true;
        static const bool m_addReset = true;

        SpatialSphericalTimeRandomForceField(unsigned int seed,
                                           PREC boostTime,
                                           PREC pauseTime,
                                           PREC startTime,
                                           PREC endTime,
                                           PREC amplitude,
                                           AABB randomBox,
                                           bool randomOn
                                           ):
            m_boostTime(boostTime),
            m_pauseTime(pauseTime),
            m_startTime(startTime),
            m_endTime(endTime),
            m_seed(seed), m_amplitude(amplitude), m_randomBox(randomBox), m_randomOn(randomOn)
        {
            m_randomG = nullptr;
            reset();
        }

        ~SpatialSphericalTimeRandomForceField(){
            delete m_randomG;
        }

        template<typename TRigidBody>
        void calculate(TRigidBody * body){
            if(m_inInterval){
                ASSERTMSG(body->m_pSolverData, "Solverdata not present!")
                if(m_ts <= m_boostTime){
                    Vector3 r = body->m_r_S - m_offset ;
                    r.normalize();
                    body->m_h_term.template head<3>() += r*m_amplitude;
                }
            }
        }



        void setTime(PREC time){
            m_t = time;

            if(m_inInterval == false){
                if(m_t >= m_startTime && m_t <= m_endTime){
                    m_inInterval = true;
                }
            }

            if(m_inInterval == true){
                if(m_t < m_startTime || m_t > m_endTime){
                    m_inInterval = false;
                    return;
                }


                m_ts = std::fmod((m_t-m_startTime), (m_boostTime+m_pauseTime)) ;

                if(m_randomOn){
                    if(m_newPos == false && m_ts>m_boostTime){
                        Vector3 ex = m_randomBox.extent();
                        m_offset = m_randomBox.m_minPoint + Vector3( ex(0)* (*m_randomG)(),  ex(1) * (*m_randomG)() , ex(2) * (*m_randomG)() );
                        m_newPos = true;
                    }else if( m_newPos == true && m_ts<=m_boostTime){
                        m_newPos = false;
                    }
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
            m_inInterval = false;
        }



    private:

        SpatialSphericalTimeRandomForceField(const SpatialSphericalTimeRandomForceField &);

        boost::variate_generator< boost::mt19937 , boost::uniform_real<PREC> > * m_randomG;
        PREC m_boostTime, m_pauseTime, m_amplitude, m_t, m_ts, m_startTime, m_endTime;
        Vector3 m_offset;
        bool m_newPos, m_randomOn, m_inInterval;
        AABB m_randomBox;
        unsigned int m_seed ;


};


#include RigidBody_INCLUDE_FILE

class ExternalForceList{
    public:
        DEFINE_DYNAMICSSYTEM_CONFIG_TYPES

        typedef typename std::vector< std::function<void (RigidBodyType *)> >::iterator iterator;

        // Wants a new pointer, it takes care of deleting the objects!
        template<typename T>
        void addExternalForceCalculation(T * extForce){

            m_deleterList.push_back( ExternalForceList::DeleteFunctor<T>(extForce) );

            if(extForce->m_addReset){
                m_resetList.push_back( std::bind( &T::reset, extForce ) );
            }

            if(extForce->m_addCalculate){
                m_calculationList.push_back( std::bind(&T::template calculate<RigidBodyType>, extForce, std::placeholders::_1 ) );
            }

            if(extForce->m_addSetTime){
                m_setTimeList.push_back( std::bind(&T::setTime, extForce, std::placeholders::_1 ) );
            }
        }

        ~ExternalForceList(){
            for(auto & f : m_deleterList){ f();} // delete all objects
        }

        inline void reset(){
            for(auto & f : m_resetList){f();} // reseter list
        }

        inline void setTime(PREC time){
            for(auto & f : m_setTimeList){f(time);} // setTime
        }

        void calculate(RigidBodyType * body){
            for(auto & f : m_calculationList){
                f(body); // Apply calculation function!
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

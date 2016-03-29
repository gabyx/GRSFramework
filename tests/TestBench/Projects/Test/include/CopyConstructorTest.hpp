// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#include <functional>
#include <cmath>

#include "boost/random.hpp"
#include "boost/generator_iterator.hpp"

template<typename RigidBodyType>
class SpatialUniformTimeRandomForceField{
    public:
        typedef double PREC;

        SpatialUniformTimeRandomForceField(unsigned int seed, PREC boostTime, PREC pauseTime, PREC amplitude):
            m_boostTime(boostTime),m_pauseTime(pauseTime),m_seed(seed), m_amplitude(amplitude)
        {
            m_randomG = NULL;
            reset();
        }

        ~SpatialUniformTimeRandomForceField(){
            if(m_randomG){
                delete m_randomG;
            }
        }
        void calculate(RigidBodyType * body){

            std::cout << "FUNCTION CALL" << std::endl;
//                m_offsetX = (*m_randomG)();
//                m_offsetY = (*m_randomG)();
//                m_offsetZ = (*m_randomG)();


        }

        void reset(){
            if(m_randomG){
                delete m_randomG;
            }

            boost::mt19937  generator(m_seed);
            boost::uniform_real<PREC> uniform(-1,1);
            m_randomG  = new boost::variate_generator< boost::mt19937 , boost::uniform_real<PREC> >(generator, uniform);

            m_offsetX=0;
            m_offsetY=0;
            m_offsetZ=0;
        }

//        SpatialUniformTimeRandomForceField(const SpatialUniformTimeRandomForceField &){
//            std::cout << "COPY CONSRUCTOR!" << std::endl;
//        }
    private:
        boost::variate_generator< boost::mt19937 , boost::uniform_real<PREC> > * m_randomG;
        PREC m_boostTime, m_pauseTime, m_amplitude, m_offsetX, m_offsetY, m_offsetZ ;
        unsigned int m_seed ;


};


void copyConstructorTest(){
    SpatialUniformTimeRandomForceField<int> * ptr = new SpatialUniformTimeRandomForceField<int>(1,1,1,1);

   std::function< void(int*) > func = std::bind( &SpatialUniformTimeRandomForceField<int>::calculate , ptr , std::placeholders::_1);

   int A;
   func(&A);
};

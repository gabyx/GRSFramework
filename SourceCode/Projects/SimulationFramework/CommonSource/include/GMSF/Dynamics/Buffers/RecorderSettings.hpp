#ifndef GMSF_Dynamics_Buffers_RecorderSettings_hpp
#define GMSF_Dynamics_Buffers_RecorderSettings_hpp

#include "TypeDefs.hpp"


/**
* @ingroup DynamicsGeneral
* @brief The recorder settings.
*/

class RecorderSettings {
public:
    DEFINE_LAYOUT_CONFIG_TYPES

    RecorderSettings() {
        m_eMode = RECORD_EVERY_STEP;
        m_recordEveryXTimestep = 1;
    }
    enum RecorderMode {
        RECORD_EVERY_STEP,
        RECORD_EVERY_X_STEP, ///< Set the number of how many steps to skip
        RECORD_NOTHING
    };

    bool outputCheck(unsigned int iterationCount){
        if(m_eMode == RECORD_EVERY_X_STEP){
        //std:: cout << iterationCount <<","<<m_recordEveryXTimestep<<std::endl;
             if (iterationCount % m_recordEveryXTimestep == 0){
                return true;
             }
        }
        else if(m_eMode == RECORD_EVERY_STEP){
            return true;
        }
        return false;
    }

    void setMode(RecorderMode mode){
        m_eMode = mode;
        if(m_eMode==RECORD_EVERY_STEP){
            m_recordEveryXTimestep = 1;
        }
    }

    RecorderMode & getMode(){
        return m_eMode;
    }

    void setEveryXTimestep(PREC stepsPerSecond, PREC timeStep){
        int everyXStep = std::floor(1.0 /(stepsPerSecond * timeStep));
        if(everyXStep<1){
           m_recordEveryXTimestep = 1;
        }else{
            m_recordEveryXTimestep = everyXStep;
        }
    }

    unsigned int getEveryXTimestep(){return m_recordEveryXTimestep;}

    private:
    RecorderMode m_eMode;
    unsigned int	m_recordEveryXTimestep;
};

#endif


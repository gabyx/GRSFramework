#ifndef GRSF_Logic_LineWriter_hpp
#define GRSF_Logic_LineWriter_hpp

#include <fstream>
#include "GRSF/Logic/LogicNode.hpp"

namespace LogicNodes{

    template<typename TValue>
    class LineWriter : public LogicNode  {
    public:

        struct Inputs {
            enum {
                Value,
                INPUTS_LAST
            };
        };

        struct Outputs {
            enum {
                OUTPUTS_LAST
            };
        };

        enum {
            N_INPUTS  = Inputs::INPUTS_LAST,
            N_OUTPUTS = Outputs::OUTPUTS_LAST,
            N_SOCKETS = N_INPUTS + N_OUTPUTS
        };


        DECLARE_ISOCKET_TYPE(Value, TValue );

        LineWriter(unsigned int id,const boost::filesystem::path & scriptPath):
            LogicNode(id), m_scriptPath(scriptPath)
        {
            if(boost::filesystem::exists(m_scriptPath)) {
                ERRORMSG("Output script file LineWriter at: " << m_scriptPath << " exists!")
            }

            m_script.open(m_scriptPath.string(), std::ios::trunc);

            if(! m_script.good()){
                ERRORMSG("Output script file LineWriter at: " << m_scriptPath << " could not be opened!")
            }
            ADD_ISOCK(Value,TValue());
        }

        void compute() {
            m_script << GET_ISOCKET_VALUE(Value) << std::endl;
        }

        ~LineWriter(){}
    private:
        boost::filesystem::path m_scriptPath;
        std::ofstream m_script;
    };

};

#endif

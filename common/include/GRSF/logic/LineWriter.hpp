#ifndef GRSF_logic_LineWriter_hpp
#define GRSF_logic_LineWriter_hpp

#include <fstream>
#include "GRSF/logic/LogicNode.hpp"

namespace LogicNodes{

    template<typename TValue>
    class LineWriter : public LogicNode  {
    public:

        struct Inputs {
            enum {
                File,
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

        DECLARE_ISOCKET_TYPE(File, boost::filesystem::path );
        DECLARE_ISOCKET_TYPE(Value, TValue );


        LineWriter(unsigned int id,const boost::filesystem::path filePath = "", bool truncate = true):
            LogicNode(id), m_truncate(truncate)
        {
            ADD_ISOCK(File,filePath);
            ADD_ISOCK(Value,TValue());
        }

        void compute() {
            if( GET_ISOCKET_VALUE(File) != m_openedFile || !m_file.is_open() ){
                m_openedFile = GET_ISOCKET_VALUE(File);
                openFile(m_openedFile);
            }
            m_file << GET_ISOCKET_VALUE(Value) << std::endl;
        }

        void openFile(const boost::filesystem::path & f){
            // file path has changes, close file and open new one
            if(m_file.is_open()){
                m_file.close();
            }
            if(m_truncate){
                m_file.open(f.string(), std::ios::trunc);
            }else{
                m_file.open(f.string(), std::ios::app);
            }
            if(! m_file.good()){
                ERRORMSG("Output script file LineWriter at: " << f << " could not be opened!")
            }
        }

        ~LineWriter(){
            if(m_file.is_open()){
                m_file.close();
            }
        }
    private:
        boost::filesystem::path m_openedFile = "";
        std::ofstream m_file;
        bool m_truncate = true;
    };

};

#endif

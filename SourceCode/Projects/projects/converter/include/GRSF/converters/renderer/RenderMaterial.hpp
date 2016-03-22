#ifndef GRSF_General_RenderMaterial_hpp
#define GRSF_General_RenderMaterial_hpp

#include <sstream>
#include "GRSF/Common/TypeDefs.hpp"

class RenderMaterial{
    public:
        RenderMaterial():m_id(0){}
        RenderMaterial(unsigned int id, const std::string & s): m_id(id)
        {
            m_s << s;
        }

        ~RenderMaterial(){};

        template<typename T>
        inline RenderMaterial & operator<<(const T & t){
            m_s << t;
            return *this;
        }

        inline std::string str(){return m_s.str();}

        inline void clear(){
           m_s.str("");
        }

        inline void write(std::stringstream & s){
            s << m_s.rdbuf() << "\n";
            s.clear();
            m_s.seekp(0);
            m_s.seekg(0);
        }

    private:
        std::stringstream m_s;
        unsigned int m_id;
    };

#endif

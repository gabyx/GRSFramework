    #ifndef RenderMaterial_hpp
    #define RenderMaterial_hpp

    #include "TypeDefs.hpp"

    class RenderMaterial{
    public:
        RenderMaterial(unsigned int id, const std::string & s): m_id(id), m_s(s){}
        ~RenderMaterial(){};

        void setMaterialString(const std::string & s){m_s = s;}

        std::string & getMaterialString(){
            return m_s;
        }
    private:
        std::string m_s;
        unsigned int m_id;
    };

    #endif

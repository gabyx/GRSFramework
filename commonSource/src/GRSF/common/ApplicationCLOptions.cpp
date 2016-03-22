#include "GRSF/Common/ApplicationCLOptions.hpp"

std::ostream & operator<<(std::ostream & s, const ApplicationCLOptions::PostProcessTask & p){
            s << " Postprocess: " << p.m_name << " : ";
            s << (p.m_options.begin())->first << ":\""<<(p.m_options.begin())->second <<"\"";
            for(auto it= ++(p.m_options.begin());it != p.m_options.end(); it++){
                s <<", " <<it->first<< ":\""<<it->second <<"\"";
            }
            return s;
};


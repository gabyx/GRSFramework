#ifndef GRSF_Common_DemangleTypes_hpp
#define GRSF_Common_DemangleTypes_hpp

#include <typeinfo>
#include <string>

namespace demangle{

    namespace details{
        std::string demangle(const char* name);
    };

    template <class T>
    std::string type(const T& t) {
        return details::demangle(typeid(t).name());
    }

    template <class T>
    std::string type() {
        return details::demangle(typeid(T).name());
    }

};

#endif

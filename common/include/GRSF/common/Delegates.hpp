#ifndef GRSF_common_Delegates_hpp
#define GRSF_common_Delegates_hpp

#include <functional>
#include <fastfunc/fastfunc.hpp> // Use FastFunc


namespace Delegates{

    /**
    * This file defines the used delegates in this framework
    */

    template<typename T> using DelegateFunc = typedef ssvu::FastFunc<T>;
    //template<typename T> using DelegateFunc = typedef std::function<T>;
}



#endif // Delegates_hpp

#ifndef GRSF_Common_EnumClassHelper_hpp
#define GRSF_Common_EnumClassHelper_hpp

#include <type_traits>

namespace EnumConversion{
    /** This function casts any enum class to the underlying type */
    template<typename E>
    constexpr auto toIntegral(const E e) -> typename std::underlying_type<E>::type
    {
       return static_cast<typename std::underlying_type<E>::type>(e);
    }
};

#endif // EnumClassHelper_hpp




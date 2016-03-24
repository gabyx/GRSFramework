#ifndef GRSF_common_SerializationHelpersTuple_hpp
#define GRSF_common_SerializationHelpersTuple_hpp

#include <tuple>
#include "GRSF/common/SfinaeMacros.hpp"

namespace grsf {
    namespace details{

        template<typename Archive,
                 typename ... Elements,
                 std::size_t N = sizeof...(Elements),
                 SFINAE_ENABLE_IF(N>1)
        >
        inline void serialize(Archive& ar, std::tuple<Elements...>& t) {
           ar & std::get<N-1>(t);
           serialize<N-1>(ar, t);
        }

        template<typename Archive,
                 typename ... Elements,
                 std::size_t N = sizeof...(Elements),
                 SFINAE_ENABLE_IF(N==1)
        >
        inline void serialize(Archive& ar, std::tuple<Elements...>& t) {
           ar & std::get<N-1>(t);
        }

    };
};

namespace boost {
  namespace serialization {

    template<typename Archive, typename ... Elements>
    Archive& serialize(Archive& ar, std::tuple<Elements...>& t, const unsigned int version) {
        grsf::details::serialize(ar, t);
        return ar;
    }

  };
};

#endif

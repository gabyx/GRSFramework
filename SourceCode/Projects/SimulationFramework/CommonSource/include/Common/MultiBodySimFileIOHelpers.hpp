#ifndef MultiBodySimFileIOHelpers_hpp
#define MultiBodySimFileIOHelpers_hpp

#include <Eigen/Dense>

namespace IOHelpers{

    template<typename Derived>
    std::ostream & operator<<( std::ostream &os, const Eigen::EigenBase<Derived>  & m){
        os.write(reinterpret_cast<char const *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
        return os;
    }

    template<typename Derived>
    std::istream &operator>>( std::istream &os, Eigen::EigenBase<Derived>  & m){
        os.read(reinterpret_cast<char *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
        return os;
    }
};

#endif

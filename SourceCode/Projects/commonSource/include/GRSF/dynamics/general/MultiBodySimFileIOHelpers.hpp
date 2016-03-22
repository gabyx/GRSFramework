#ifndef GRSF_Dynamics_General_MultiBodySimFileIOHelpers_hpp
#define GRSF_Dynamics_General_MultiBodySimFileIOHelpers_hpp

#include <Eigen/Dense>

namespace IOHelpers{

//    template<typename Derived>
//    std::ostream & operator<<( std::ostream &os, const Eigen::EigenBase<Derived>  & m){
//        os.write(reinterpret_cast<char const *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
//        return os;
//    }
//
//    template<typename Derived>
//    std::istream &operator>>( std::istream &os, Eigen::EigenBase<Derived>  & m){
//        os.read(reinterpret_cast<char *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
//        return os;
//    }

    template<typename Derived>
    void writeBinary(std::ostream &os, const Eigen::EigenBase<Derived>  & m)
    {
        os.write(reinterpret_cast<char const *>(m.derived().data()), m.size()*sizeof(typename Derived::Scalar));
    }

    template<typename Derived>
    void readBinary(std::istream &os, const Eigen::EigenBase<Derived>  & m)
    {
        os.read(reinterpret_cast<char *>(
                                         const_cast<typename Derived::Scalar*>(m.derived().data())),
                                         m.size()*sizeof(typename Derived::Scalar));
    }
};

#endif

#ifndef MultiBodySimFileIOHelpers_hpp
#define MultiBodySimFileIOHelpers_hpp

#include <Eigen/Dense>

namespace IOHelpers{
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

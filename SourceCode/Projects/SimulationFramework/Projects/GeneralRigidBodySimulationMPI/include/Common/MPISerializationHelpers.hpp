#ifndef MPISerializationHelpers_hpp
#define MPISerializationHelpers_hpp

#include <Eigen/Dense>
#include <boost/serialization/array.hpp>




template<class Archive, typename Derived>
void serializeEigen(Archive & ar, Eigen::EigenBase<Derived> & g) {
//            std::cout << "Serialize Eigen Object:"<<std::endl;
//            std::cout << "   Size: " << g.size()<<std::endl;
//            for(int i=0;i<g.size();i++){
//                ar & *(g.derived().data() + i);
//            }
    ar & boost::serialization::make_array(g.derived().data(), g.size());
};

template<class Archive, typename Derived>
void serializeEigen(Archive & ar, const Eigen::EigenBase<Derived> & gc) {
//            std::cout << "Serialize Eigen Object:"<<std::endl;
//            std::cout << "   Size: " << g.size()<<std::endl;
//            for(int i=0;i<g.size();i++){
//                ar & *(g.derived().data() + i);
//            }
    Eigen::EigenBase<Derived> & g = const_cast<Eigen::EigenBase<Derived> &>(gc);
    ar & boost::serialization::make_array(g.derived().data(), g.size());
};



namespace boost {
namespace serialization {

    template<class Archive, typename PREC>
    void serialize(Archive & ar, BoxGeometry<PREC> & g, const unsigned int version) {
        serializeEigen(ar,g.m_extent);
        serializeEigen(ar,g.m_center);
    }

    template<class Archive, typename PREC>
    void serialize(Archive & ar, HalfspaceGeometry<PREC> & g, const unsigned int version) {

        serializeEigen(ar,g.m_normal);
        serializeEigen(ar,g.m_pos);
    }

    template<class Archive, typename PREC>
    void serialize(Archive & ar, SphereGeometry<PREC> & g, const unsigned int version) {

        ar & g.m_radius;

    }

    template<class Archive, typename PREC>
    void serialize(Archive & ar, MeshGeometry<PREC> & g, const unsigned int version) {
        ERRORMSG("No implementation for MeshGeometry serialization!");
    }

};
};

#endif

#ifndef MPISerializationHelpers_hpp
#define MPISerializationHelpers_hpp

#include <Eigen/Dense>
#include <boost/serialization/array.hpp>




template<class Archive, typename Derived>
void serializeEigen(Archive & ar, Eigen::EigenBase<Derived> & g, const unsigned int version)
{
//            std::cout << "Serialize Eigen Object:"<<std::endl;
//            std::cout << "   Size: " << g.size()<<std::endl;
//            for(int i=0;i<g.size();i++){
//                ar & *(g.derived().data() + i);
//            }
    ar & boost::serialization::make_array(g.derived().data(), g.size());
};




#endif

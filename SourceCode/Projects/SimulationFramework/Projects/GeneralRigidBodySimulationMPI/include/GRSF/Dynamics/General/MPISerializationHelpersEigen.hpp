#ifndef GRSF_Dynamics_General_MPISerializationHelpersEigen_hpp
#define GRSF_Dynamics_General_MPISerializationHelpersEigen_hpp

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




namespace boost{
    namespace serialization{

        template<   class Archive,
                    class S,
                    int Rows_,
                    int Cols_,
                    int Ops_,
                    int MaxRows_,
                    int MaxCols_>
        inline void save(
            Archive & ar,
            const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
            const unsigned int version)
            {
                int rows = g.rows();
                int cols = g.cols();

                ar & rows;
                ar & cols;
                ar & boost::serialization::make_array(g.data(), rows * cols);
            }

        template<   class Archive,
                    class S,
                    int Rows_,
                    int Cols_,
                    int Ops_,
                    int MaxRows_,
                    int MaxCols_>
        inline void load(
            Archive & ar,
            Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
            const unsigned int version)
        {
            int rows, cols;
            ar & rows;
            ar & cols;
            g.resize(rows, cols);
            ar & boost::serialization::make_array(g.data(), rows * cols);
        }

        template<   class Archive,
                    class S,
                    int Rows_,
                    int Cols_,
                    int Ops_,
                    int MaxRows_,
                    int MaxCols_>
        inline void serialize(
            Archive & ar,
            Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
            const unsigned int version)
        {
            split_free(ar, g, version);
        }


    } // namespace serialization
} // namespace boost

#endif // MPISerializationHelpersEigen_hpp

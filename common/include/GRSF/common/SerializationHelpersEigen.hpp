// ========================================================================================
//  GRSFramework
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com>
//
//  This Source Code Form is subject to the terms of the GNU General Public License as
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_common_SerializationHelpersEigen_hpp
#define GRSF_common_SerializationHelpersEigen_hpp

#include <Eigen/Dense>
#include <boost/serialization/array.hpp>

template <class Archive, typename Derived>
void serializeEigen(Archive& ar, Eigen::EigenBase<Derived>& g)
{
    //            std::cout << "Serialize Eigen Object:"<<std::endl;
    //            std::cout << "   Size: " << g.size()<<std::endl;
    //            for(int i=0;i<g.size();i++){
    //                ar & *(g.derived().data() + i);
    //            }
    ar& boost::serialization::make_array(g.derived().data(), g.size());
};

template <class Archive, typename Derived>
void serializeEigen(Archive& ar, const Eigen::EigenBase<Derived>& gc)
{
    //            std::cout << "Serialize Eigen Object:"<<std::endl;
    //            std::cout << "   Size: " << g.size()<<std::endl;
    //            for(int i=0;i<g.size();i++){
    //                ar & *(g.derived().data() + i);
    //            }
    Eigen::EigenBase<Derived>& g = const_cast<Eigen::EigenBase<Derived>&>(gc);
    ar&                        boost::serialization::make_array(g.derived().data(), g.size());
};

namespace boost
{
namespace serialization
{
template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void save(Archive& ar,
                 const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                 const unsigned int version)
{
    int rows = g.rows();
    int cols = g.cols();

    ar& rows;
    ar& cols;
    ar& boost::serialization::make_array(g.data(), rows * cols);
}

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void load(Archive& ar, Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g, const unsigned int version)
{
    int rows, cols;
    ar& rows;
    ar& cols;
    g.resize(rows, cols);
    ar& boost::serialization::make_array(g.data(), rows * cols);
}

template <class Archive, class S, int Rows_, int Cols_, int Ops_, int MaxRows_, int MaxCols_>
inline void serialize(Archive& ar,
                      Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_>& g,
                      const unsigned int version)
{
    split_free(ar, g, version);
}

}  // namespace serialization
}  // namespace boost

#endif

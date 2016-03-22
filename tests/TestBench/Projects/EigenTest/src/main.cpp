
#include <iostream>
#include <type_traits>


#include <Eigen/Dense>
//#include "EigenTensorTest.hpp"




int main(int, char**)
{

    const double b[2] = {1,2};

    //eigenTensorTest();
    using Vec2 = Eigen::Matrix<double,2,1>;

    Eigen::Map<const Vec2> m(b);

    using T = Eigen::Matrix<double,2,1>;

    static_assert( (std::is_base_of< Eigen::DenseBase<T> , T >::value), "" );


return 0;
}


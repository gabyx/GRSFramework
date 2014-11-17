#include "MyMatrixDefs.hpp"

Eigen::IOFormat MyMatrixIOFormat::Matlab(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
Eigen::IOFormat MyMatrixIOFormat::CommaSep(Eigen::FullPrecision, 0, ", ", "\n", "", "", "", "");
Eigen::IOFormat MyMatrixIOFormat::SpaceSep(Eigen::FullPrecision, 0, "", "\n", "", "", "", "");

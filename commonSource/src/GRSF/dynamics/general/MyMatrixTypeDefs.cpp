#include "GRSF/Dynamics/General/MyMatrixTypeDefs.hpp"

Eigen::IOFormat MyMatrixIOFormat::Matlab(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");
Eigen::IOFormat MyMatrixIOFormat::CommaSep(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
Eigen::IOFormat MyMatrixIOFormat::SpaceSep(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n", "", "", "", "");

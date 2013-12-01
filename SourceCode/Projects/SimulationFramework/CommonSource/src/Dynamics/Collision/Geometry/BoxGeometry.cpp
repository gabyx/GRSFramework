#include "BoxGeometry.hpp"
// Easy static char to build easily the points!
char BoxGeometry::m_pointIdx[8*3] ={ 1,1,1,
                                            -1,1,1,
                                            1,-1,1,
                                            -1,-1,1,
                                            1,1,-1,
                                            -1,1,-1,
                                            1,-1,-1,
                                            -1,-1,-1};

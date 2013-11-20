#ifndef DynamicCoordinateFrames_hpp
#define DynamicCoordinateFrames_hpp

#include "DynamicLines.hpp"

class DynamicCoordinateFrames{
public:

    DynamicCoordinateFrames();
    ~DynamicCoordinateFrames();

private:

    DynamicLines m_XAxis;
    DynamicLines m_YAxis;
    DynamicLines m_ZAxis;

};

#endif

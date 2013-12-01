#ifndef HalfspaceGeometry_hpp
#define HalfspaceGeometry_hpp

#include <TypeDefs.hpp>
#include <boost/serialization/access.hpp>

#include "PlaneGeometry.hpp"

class HalfspaceGeometry : public PlaneGeometry {
public:

    DEFINE_LAYOUT_CONFIG_TYPES

    HalfspaceGeometry(): PlaneGeometry() {};
    HalfspaceGeometry( const Vector3 & n, const Vector3 & p ): PlaneGeometry(n,p){};

private:
    friend class boost::serialization::access;

};


#endif

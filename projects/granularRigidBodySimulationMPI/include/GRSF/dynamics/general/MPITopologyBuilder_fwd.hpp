#ifndef GRSF_Dynamics_General_MPITopologyBuilder_fwd_hpp
#define GRSF_Dynamics_General_MPITopologyBuilder_fwd_hpp

#include <type_traits>

namespace MPILayer{

    /** Forward declaration of TopologyBuilder types */
    template<typename T> class GridTopologyBuilder;
    template<typename T> class KdTreeTopologyBuilder;

    /** Some helpers to distinguish between the topo builders */
    template<typename TopoType> struct isGridTopoBuilder;

    template<typename T>
    struct  isGridTopoBuilder< GridTopologyBuilder<T> > {
        static const bool value = true;
    };

    template<typename TopoType> struct isKdTreeTopoBuilder;

    template<typename T>
    struct  isKdTreeTopoBuilder< KdTreeTopologyBuilder<T> > {
        static const bool value = true;
    };

};
#endif

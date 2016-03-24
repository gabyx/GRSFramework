// ========================================================================================
//  GRSFramework 
//  Copyright (C) 2016 by Gabriel Nützi <gnuetzi (at) gmail (døt) com> 
// 
//  This Source Code Form is subject to the terms of the GNU General Public License as 
//  published by the Free Software Foundation; either version 3 of the License,
//  or (at your option) any later version. If a copy of the GPL was not distributed with
//  this file, you can obtain one at http://www.gnu.org/licenses/gpl-3.0.html.
// ========================================================================================

#ifndef GRSF_dynamics_general_MPITopologyBuilder_fwd_hpp
#define GRSF_dynamics_general_MPITopologyBuilder_fwd_hpp

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

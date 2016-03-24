#ifndef GRSF_dynamics_general_MPICommunicatorId_hpp
#define GRSF_dynamics_general_MPICommunicatorId_hpp

/**
* Identifiers for common used communicators
*/
namespace MPILayer{
    enum class MPICommunicatorId: unsigned int{
            WORLD_COMM = 0,
            SIM_COMM = 1,                 ///< All processes which take part in the simulation

            /** Only processes which take part in the convergence comm
            * The underlying communicator for this Id can be different for each process!
            */
            INCLUSION_CONVERGENCE_COMM = 2
    };
};

#endif

#!/bin/bash
# source this files to make available certain macros defined below


echo "GRSFramework:: commands: \n \t run_sim(_d), run_simgui(_d), run_simmpi(_d(_asan)) , run_simconv(_d) \n \t are now available."

export SIMMPI_D="${GRSF_REPO_BUILD_DIR}/debug/projects/granularRigidBodySimulationMPI/bin/debug/GRSFSimMPI"
export SIMGUI_D="${GRSF_REPO_BUILD_DIR}/debug/projects/granularRigidBodySimulationGUI/bin/debug/GRSFSimGUI"
export SIMNOGUI_D="${GRSF_REPO_BUILD_DIR}/debug/projects/granularRigidBodySimulation/bin/debug/GRSFSim"
export SIMCONV_D="${GRSF_REPO_BUILD_DIR}/debug/projects/converter/bin/debug/GRSFConverter"

export SIMMPI_D="${GRSF_REPO_BUILD_DIR}/release/projects/granularRigidBodySimulationMPI/bin/release/GRSFSimMPI"
export SIMGUI_D="${GRSF_REPO_BUILD_DIR}/release/projects/granularRigidBodySimulationGUI/bin/release/GRSFSimGUI"
export SIMNOGUI_D="${GRSF_REPO_BUILD_DIR}/release/projects/granularRigidBodySimulation/bin/release/GRSFSim"
export SIMCONV_D="${GRSF_REPO_BUILD_DIR}/release/projects/converter/bin/release/GRSFConverter"

# pipe asanMangler to the any GRSF executable to trace output from address and leak sanitizer
export ASANMANGLER="${GRSF_SIMULATION_DIR}/python/scripts/AsanSymbolize/asan_symbolize.py"
alias asanMangler="${ASANMANGLER}"

alias run_sim="${SIMNOGUI}"
alias run_simgui="${SIMGUI}"
alias run_simconv="${SIMCONV}"

alias run_sim_d="${SIMNOGUI_D}"
alias run_simgui_d="${SIMGUI_D}"
alias run_simconv_d="${SIMCONV_D}"


run_simmpi() {
   n=$1; shift;
   eval mpiexec -np $n $SIMMPI "$@"
}

run_simmpi_d() {
    n=$1; shift;
   eval mpiexec -np $n $SIMMPI_D "$@" 
}

run_simmpi_d_asan() {
    n=$1; shift;
   eval mpiexec -np $n $SIMMPI_D "$@" 2>&1 | asanMangler
}

export -f run_simmpi
export -f run_simmpi_d



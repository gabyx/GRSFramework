#!/bin/bash
# source this files to make available certain macros defined below


echo "GRSFramework:: commands: \n \t run_sim(_d), run_simgui(_d), run_simmpi(_d(_asan)) , run_simconv(_d) \n \t are now available."

export SIMMPI_D="${GRSF_REPO_BUILD_DIR}/Debug/Projects/GeneralRigidBodySimulationMPI/bin/Debug/GeneralRigidBodySimulationMPI"
export SIMGUI_D="${GRSF_REPO_BUILD_DIR}/Debug/Projects/GeneralRigidBodySimulation/bin/Debug/GeneralRigidBodySimulation"
export SIMNOGUI_D="${GRSF_REPO_BUILD_DIR}/Debug/Projects/GeneralRigidBodySimulationNoGUI/bin/Debug/GeneralRigidBodySimulationNoGUI"
export SIMCONV_D="${GRSF_REPO_BUILD_DIR}/Debug/Projects/Converter/bin/Debug/Converter"

export SIMMPI="${GRSF_REPO_BUILD_DIR}/Release/Projects/GeneralRigidBodySimulationMPI/bin/Release/GeneralRigidBodySimulationMPI"
export SIMGUI="${GRSF_REPO_BUILD_DIR}/Release/Projects/GeneralRigidBodySimulation/bin/Release/GeneralRigidBodySimulation"
export SIMNOGUI="${GRSF_REPO_BUILD_DIR}/Release/Projects/GeneralRigidBodySimulationNoGUI/bin/Release/GeneralRigidBodySimulationNoGUI"
export SIMCONV="${GRSF_REPO_BUILD_DIR}/Release/Projects/Converter/bin/Release/Converter"

# pipe asanMangler to the any GRSF executable to trace output from address and leak sanitizer
export ASANMANGLER="${GRSF_SIMULATION_DIR}/python/Tools/AsanSymbolize/asan_symbolize.py"
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



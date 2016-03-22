#!/bin/bash


export SIMMPI_D="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Debug/Projects/GeneralRigidBodySimulationMPI/bin/Debug/GeneralRigidBodySimulationMPI"
export SIMGUI_D="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Debug/Projects/GeneralRigidBodySimulation/bin/Debug/GeneralRigidBodySimulation"
export SIMNOGUI_D="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Debug/Projects/GeneralRigidBodySimulationNoGUI/bin/Debug/GeneralRigidBodySimulationNoGUI"
export SIMCONV_D="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Debug/Projects/Converter/bin/Debug/Converter"

export SIMMPI="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Release/Projects/GeneralRigidBodySimulationMPI/bin/Release/GeneralRigidBodySimulationMPI"
export SIMGUI="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Release/Projects/GeneralRigidBodySimulation/bin/Release/GeneralRigidBodySimulation"
export SIMNOGUI="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Release/Projects/GeneralRigidBodySimulationNoGUI/bin/Release/GeneralRigidBodySimulationNoGUI"
export SIMCONV="${ROOT_BUILD_DIR}/SIMULATION_FRAMEWORK/Release/Projects/Converter/bin/Release/Converter"


export MPITEST="${ROOT_BUILD_DIR}/MPI_TESTS/Debug/Projects/MpiTests/MpiTests"
export TESTBENCH="${ROOT_BUILD_DIR}/TEST_BENCH/Projects/Test/Test"

alias run_simnogui="${SIMNOGUI}"
alias run_sim="${SIMGUI}"
alias run_simconv="${SIMCONV}"

alias run_simnogui_d="${SIMNOGUI_D}"
alias run_sim_d="${SIMGUI_D}"
alias run_simconv_d="${SIMCONV_D}"


run_simmpi() {
   n=$1; shift;
   eval mpiexec -np $n $SIMMPI "$@"
}

run_simshowargs() {
   eval mpiexec -np $1 ./ShowAllArgs/ShowAllArgs $2  
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



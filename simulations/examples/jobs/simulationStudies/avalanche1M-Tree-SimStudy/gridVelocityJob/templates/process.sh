#!/bin/bash


# Source all common functions
source ${General:configuratorModuleDir}/jobGenerators/jobGeneratorMPI/scripts/commonFunctions.sh

if [[ -z "${Job:processIdxVariabel}" ]]; then
    echo "Rank not defined! "
    exitFunction 111
fi

function ES(){ echo "$(currTime) :: process.sh: Rank: ${Job:processIdxVariabel}"; }

executionDir=$(pwd)
thisPID="$BASHPID"
stage=0
signalReceived="False"
cleaningUp="False"


function executeFileValidation(){
    # assemble pipeline status
    PYTHONPATH=${General:configuratorModulePath}
    export PYTHONPATH

    python -m HPCJobConfigurator.jobGenerators.jobGeneratorMPI.generatorToolPipeline.scripts.generateFileValidation  \
        --searchDirNew="${processDir}" \
        --pipelineSpecs="${Pipeline:pipelineSpecs}" \
        --validateOnlyLastModified=True \
        --output="${Pipeline:validationInfoFile}"
        
    return $?
}

function cleanup(){
    if [[ "${cleaningUp}" == "True" ]] ; then
        # we are already cleaning up
        return 0
    else
        cleaningUp="True"
    fi
    
    echo "$(ES) do cleanup! =============" 
    echo 
    echo "Execute CleanUpCommand ${Pipeline:cleanUpCommand}"
    cd ${executionDir}
    ${Pipeline:cleanUpCommand}
    #if [[ ${stage} -ge 1 ]]; then
      #executeFileValidation
      #echo "$(ES) fileValidation exitStatus: $?"
    #fi
    echo "$(ES) cleanup finished ========"
}

function shutDownHandler() {
    # ignore all signals
    trap_with_arg ignoreAllSignals SIGINT SIGUSR1 SIGUSR2 SIGTERM
    
    signalReceived="True"
    if [[ "${cleaningUp}" == "False" ]]; then
      echo "$(ES) Signal $1 catched, cleanup and exit."
      cleanup
      exitFunction 0
    else
      echo "$(ES) Signal $1 catched, we are already cleaning up, continue."
    fi
}

# Setup the Trap
# Be aware that SIGINT and SIGTERM will be catched here, but if this script is run with mpirun
# mpirun will forward SIGINT/SIGTERM and then quit, leaving this script still running in the signal handler
trap_with_arg shutDownHandler SIGINT SIGUSR1 SIGUSR2 SIGTERM SIGPIPE

# Process folder ================================
tryNoCleanUp mkdir -p "${Job:processDir}"

# Save processDir, it might be relative! and if signal handler runs 
# using a relative path is not good
cd "${Job:processDir}"
processDir=$(pwd)
# ========================================================

# Output rerouting =======================================
# save stdout in file descriptor 4
exec 4>&1
# put stdout and stderr into logFile
logFile="${processDir}/processLog.log"
#http://stackoverflow.com/a/18462920/293195
exec 3>&1 1>>${logFile} 2>&1
# filedescriptor 3 is still connected to the console
# ========================================================


# Process Stuff ===================================================================================================================

# Set important PYTHON stuff
export PYTHONPATH="${General:configuratorModulePath}:$PYTHONPATH"

echo "File Mover =======================================================" >> $logFile
echo "Search file move process file ..." >> $logFile
fileMoverProcessFile=$( python3 -c "print(\"${Pipeline-PreProcess:fileMoverProcessFile}\".format(${Job:processIdxVariabel}))" )

if [ ! -f "$fileMoverProcessFile" ]; then
    echo "File mover process file (.xml) not found! It seems we are finished moving files! (file: $fileMoverProcessFile)" >> $logFile
    
else
    echo "File mover process file : $fileMoverProcessFile" >> $logFile
    echo "Start moving files" >> $logFile
    echo "Change directory to ${Job:processDir}" >> $logFile
    cd ${Job:processDir}

    try launchInForeground python -m HPCJobConfigurator.jobGenerators.jobGeneratorMPI.generatorToolPipeline.scripts.fileMove \
        -p "$fileMoverProcessFile"

      
fi
echo "==================================================================" >> $logFile
cd ${processDir}

echo "Converter ========================================================" >> $logFile
echo "Search converter process file ..." >> $logFile
converterProcessFile=$( python3 -c "print(\"${Pipeline:converterProcessFile}\".format(${Job:processIdxVariabel}))" )

if [ ! -f "$converterProcessFile" ]; then
    echo "Converter process file (.xml) not found! It seems we are finished converting! (file: $converterProcessFile)" >> $logFile
    
else
    echo "Converter process file : $converterProcessFile" >> $logFile
    echo "Start converting the files" >> $logFile
    echo "Change directory to ${Pipeline:converterExecutionDir}" >> $logFile

    cd ${Pipeline:converterExecutionDir}

    try launchInForeground ${Pipeline:executableConverter} gridder\
        -i $converterProcessFile \
        -s ${Pipeline:sceneFile} \
        -m ${Pipeline:mediaDir} \
        -c ${Pipeline:converterLogic}

fi
echo "==================================================================" >> $logFile

cd ${processDir}


echo "Final cleanup ====================================================" >> $logFile
cleanup
echo "================================================================== " >> $logFile


exitFunction 0 
# =================================================================================================================================================
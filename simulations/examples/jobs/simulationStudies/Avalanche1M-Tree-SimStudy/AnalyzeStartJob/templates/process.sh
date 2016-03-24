#!/bin/bash

yell() { echo "$0: $*" >&2; }
die() { yell "$*"; cleanup ; exit 111 ; }
try() { "$@" || die "cannot $*"; }


function trap_with_arg() {
    func="$1" ; shift
    for sig ; do
        trap "$func $sig" "$sig"
    done
}

function cleanup(){
    echo "process.sh: Rank: ${Job:processIdxVariabel}: do cleanup!"
    ${Pipeline:cleanUpCommand}
}

function handleSignal() {
    echo "process.sh: Rank: ${Job:processIdxVariabel}: Signal $1 catched, cleanup and exit."
    cleanup 
    exit 0
}

# Setup the Trap
trap_with_arg handleSignal SIGINT SIGTERM SIGUSR1 SIGUSR2


if [[ -z "${Job:processIdxVariabel}" ]]; then
    echo "Rank not defined! "
    exit 1
fi


rm -fr ${Job:processDir}
try mkdir -p ${Job:processDir}
cd ${Job:processDir}

logFile="${Job:processDir}/processLog.log"
:> $logFile
# Set important PYTHON stuff
export PYTHONPATH=${General:configuratorModulePath}
# matplotlib directory
export MPLCONFIGDIR=${Job:processDir}/temp/matplotlib

executeFileMove(){
    python -m HPCJobConfigurator.jobGenerators.jobGeneratorMPI.generatorToolPipeline.scripts.fileMove \
        -p "$fileMoverProcessFile" >> $logFile 2>&1 
    return $?
}


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

    begin=$(date +"%s")
    try executeFileMove
    termin=$(date +"%s")
    difftimelps=$(($termin-$begin))
    echo "File mover statistics: $(($difftimelps / 60)) minutes and $(($difftimelps % 60)) seconds elapsed." >> $logFile  
      
fi
echo "==================================================================" >> $logFile
cd ${Job:processDir}

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
    begin=$(date +"%s")
    try ${Pipeline:executableConverter} analyzer\
        -i $converterProcessFile \
        -s ${Pipeline:sceneFile} \
        -m ${Pipeline:mediaDir} \
        -c ${Pipeline:converterLogic} >> $logFile 2>&1 
    
    termin=$(date +"%s")
    difftimelps=$(($termin-$begin))
    echo "Converter statistics: $(($difftimelps / 60)) minutes and $(($difftimelps % 60)) seconds elapsed." >> $logFile  
      
fi
echo "==================================================================" >> $logFile

cd ${Job:processDir}


echo "Final cleanup ====================================================" >> $logFile
cleanup
echo "================================================================== " >> $logFile


exit 0 

#!/bin/bash

echo "Enter NETHZ Password:"
stty -echo
read pass
stty echo

export SSHPASS=$pass
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.1/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/1/; beep -l 5000 ;
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.2/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/2/; beep -l 5000 ;
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.3/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/3/; beep -l 5000 ;
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.4/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/4/; beep -l 5000 ;

sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.5/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/5/; beep -l 5000 ;
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.6/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/6/; beep -l 5000 ;
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.7/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/7/; beep -l 5000 ;
sshpass -e ssh nuetzig@brutus.ethz.ch "tar czpf - -C /cluster/work/scr1/nuetzig/Avalanche1M-RunDownTrees.8/ ." | tar vxzpf - -C  /media/zfmgpu/Data/GabrielNuetzi/SimFiles/SimFilesBrutus/Avalanche1M-RunDownTrees/8/; beep -l 5000 ;
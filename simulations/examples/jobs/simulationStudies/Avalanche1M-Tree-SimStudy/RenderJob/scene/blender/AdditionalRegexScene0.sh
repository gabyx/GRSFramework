#!/bin/bash

#argument 1 :  -f make the replacement force!

shopt -s expand_aliases
source ~/.bash_aliases

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

echo "Changing directory to: $DIR"
cd "$DIR/usableRIBSequence"

# Low-Res Rendering, to EXR file, incremental rendering

fileRegexReplace \
-r 's@Format .*@Format 1600 900 1@m' \
-r 's@(Hider ((?:.*\n)*?))(?=\s*[A-Z])@Hider "raytrace" "int incremental" [1] "int minsamples" [50] "int maxsamples" [512]\n@m' \
-r 's@Display "(\+)?([^\0]+)/([^\0]+?)(\.\w*)"(.*)$@Display "\1output/\3.exr" "openexr" "rgba"@m' \
-r 's@Options "limits" "bucketsize" .*@Option "limits" "bucketsize" [16 16]@m' \
-r 's@PixelVariance .*@PixelVariance 0.01@m' \
-r 's@^[^#|\n]?[[:blank:]]*Quantize (.*)@\#Quantize \1@m' \
-R ".*Frame.*" $1 ./scene-0

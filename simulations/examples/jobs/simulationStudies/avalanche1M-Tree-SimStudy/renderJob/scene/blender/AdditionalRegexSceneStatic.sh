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
rm -r ./scene-static
cp -r ./scene-static-orig ./scene-static

# Replace Frame.rib stuff
# forma 4:3
fileRegexReplace \
`#-r 's@GranularMaterial@GranularMaterial-remove-this-tag-see-regex@g'`     `# comment that line in order to not render the granular shit` \
-r 's@Option "searchpath" .*\n@@m' \
-r 's@Format .*@Format 1440 1080 1@m' \
-r 's@(Hider ((?:.*\n)*?))(?=\s*[A-Z])@Hider "raytrace" "int incremental" [1] "int minsamples" [128] "int maxsamples" [256]\n@m' \
-r 's@Display "(\+)?([^\0]+)/([^\0]+?)(\.\w*)"(.*)$@Display "\1output/\3.exr" "openexr" "rgba"@m' \
-r 's@Options "limits" "bucketsize" .*@Option "limits" "bucketsize" [8 8]@m' \
-r 's@PixelVariance .*@PixelVariance 0.009@m' \
-r 's@^[^#|\n]?[[:blank:]]*Quantize (.*)@Quantize "rgba" 0 0 0 0@m' \
-r 's@(AreaLightSource\s+(?:.*\n)*?.*?exposure\")\s\[([0-9\.]*)\]@\1 [0.0]@m' \
-R ".*Frame.*" $1 ./scene-static

# Replace DownMaterial with GridTexture
fileRegexReplace \
-r 's@ReadArchive\s+"MaterialSlopeDown-MAT.rib"@ReadArchive "GridTexture.rib"@m' \
-R ".*CubeDown-OBJ.*" $1 ./scene-static



#!/bin/bash
#$1 filepath for recursive search and replace
#$2 perl replacment string regex (e.g: s@([^\w])_([a-zA-Z0-9]+)(?=[^\w])?@\1m_\2\3@g)

function replaceInFile(){
     /usr/bin/perl -p -i -e $2 $1
}



regex=""
ForceFlag='false'

# Processing some options =====================================================================
#echo "BEFORE GETOPT: $@";
# Execute getopt
## Note that we use `"$@"' to let each command-line parameter expand to a 
# separate word. The quotes around `$@' are essential!
# We need ARGS as the `eval set --' would nuke the return value of getopt.

ARGS=$(getopt -o r:f -l "regex:,force" -n "RenameFiles.sh" -- "$@");
# if last command failed exit
if [ $? != 0 ] ; then echo "Terminating..." >&2 ; exit 1 ; fi
# Note the quotes around `$ARGS': they are essential!
eval set -- "$ARGS"
#echo "AFTER  GETOPT: $@";
while true ; do
    case "$1" in
        -f|--force) echo "Force flag!"; ForceFlag='true'; shift ;;
        -r|--regex)  echo "Regex: "$2; regex=$2" $regex" ; shift 2;;
        --) shift ; break ;;
        *) echo "Internal error!: $1" ; exit 1 ;;
    esac
done

#echo "Remaining arguments:"
searchFolder=$1
echo "Search Folder: $1"
#=================================================================================================


echo "Replacment Strings: " "$regex"

#files=$( find $searchFolder -type f -regextype posix-extended -regex '.*\.(hpp|icc) )
files=$( find $searchFolder -type f)

echo "Files: " "$files"

for f in ${files}; do
    for r in ${regex}; do
        echo "Process file $f            with perl regex: $r"
        
        if [[ $ForceFlag == 'false' ]]; then
            echo "===== Dry run!!, add flag -f to force the run!"
        else
            replaceInFile $f $r
        fi
        
    done
done

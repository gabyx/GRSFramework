#!/bin/bash
shopt -s expand_aliases
source ~/.bash_aliases

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

pdftk DefencePresentation.pdf output uncompressed.pdf uncompress

fileRegexReplace -R ".*\.(pdf)"  \
-r 's@-configureJob@configureJob@g' -f $DIR/uncompressed.pdf


pdftk uncompressed.pdf output DefencePresentation.pdf compress
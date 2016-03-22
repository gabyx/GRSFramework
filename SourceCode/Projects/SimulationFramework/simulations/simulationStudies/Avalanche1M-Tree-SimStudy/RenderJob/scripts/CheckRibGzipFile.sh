#!/bin/bash

# terminate as early as possible if something fails
set -e

# check gzip
gzip -t $1

printf "finished"

exit 0


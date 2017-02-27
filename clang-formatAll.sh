#!/bin/bash
# format all .cpp|.hpp files in the repository

export GRSF_REPO_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

find $GRSF_REPO_DIR -type f \( -name "*.hpp" -or  -name "*.cpp" \) -and -not \( -ipath "*external*" -or -ipath "*additional*" \) | xargs clang-format -i

#!/bin/bash

################################################################################
 # @author Yongchao Wang (ycw.puzzle@hotmail.com)
 # @brief
 # @version 0.1
 # @date 2022-10-07
 #
 #@copyright Copyright (c) 2022
 #
################################################################################

set -e

#========================= options ========================#

BUILD_TYPE="Release"

#========================== usage =========================#

print_usage() {
    echo "[options]"
    echo "-t|--type         Built Type"
    echo "                  Release"
    echo "                  Debug"

    echo "[example]"
    echo "$0 -t Debug"
    exit 1
}

#===================== parse arguments ====================#

POSITIONAL=()
while [[ $# -gt 0 ]]; do
    key="$1"

    case $key in
        -t|--type)
        BUILD_TYPE=$2
        shift
        shift
        ;;

        *)
        POSITIONAL+=("$key")
        shift
        ;;
    esac
done

echo "Build Type:           ${BUILD_TYPE}"

#========================= detect =========================#

#------------------------ directories ---------------------#

SCRIPT_DIR="$( cd $(dirname $0) && pwd -P )"
BUILD_DIR="${SCRIPT_DIR}/../build/${BUILD_TYPE}"

#========================= test ==========================#

TEST_FILES="$(find ${BUILD_DIR}/bin/ -name 'ut_*' -type f -executable)"
echo $TEST_FILES

for ut in ${TEST_FILES}; do
    $ut
done

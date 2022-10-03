#!/bin/bash

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
    echo "./build.sh -t Debug"
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

#---------------------------- OS --------------------------#

OS="windows"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS="macos"
fi

#------------------------ directories ---------------------#

SCRIPT_DIR="$( cd $(dirname $0) && pwd -P )"
BUILD_DIR="${SCRIPT_DIR}/../build/${BUILD_TYPE}"

#====================== configuration =====================#

VS_GEN="Visual Studio 17 2022"

#========================= build ==========================#

mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}

if [[ "$OS" == "windows" ]]; then
    conan install "${SCRIPT_DIR}/.." -s build_type=${BUILD_TYPE}
    cmake "${SCRIPT_DIR}/.." -G "$VS_GEN" -A x64 \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
        -DTEST_DATA_DIR="${SCRIPT_DIR}/../tests/data"
    cmake --build . --config ${BUILD_TYPE} -j8
fi

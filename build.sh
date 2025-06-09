#!/usr/bin/env bash

set -e

BUILD_DIR="build"

usage() {
    echo "Uso: $0 [release|debug]"
    echo "  release  → cmake -DCMAKE_BUILD_TYPE=Release"
    echo "  debug    → cmake -DCMAKE_BUILD_TYPE=Debug"
    exit 1
}

if [ $# -ne 1 ]; then
    usage
fi

MODE="$1"
case "$MODE" in
    release|Release)
        BUILD_TYPE="Release"
        ;;
    debug|Debug)
        BUILD_TYPE="Debug"
        ;;
    *)
        usage
        ;;
esac

if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
fi

cd "$BUILD_DIR"
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" ..

cmake --build . --config "$BUILD_TYPE"

ctest --output-on-failure --config "$BUILD_TYPE"

echo
echo "Build and tests '$BUILD_TYPE' finished"

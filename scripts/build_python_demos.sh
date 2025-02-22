#!/bin/bash

function WHITE_TEXT {
  printf "\033[1;37m"
}
function NORMAL_TEXT {
  printf "\033[0m"
}
function GREEN_TEXT {
  printf "\033[1;32m"
}
function RED_TEXT {
  printf "\033[1;31m"
}

WHITE_TEXT
echo "########################################################################################"
echo "# Building Python (SWIG) Demos...                                                      #"
echo "########################################################################################"
NORMAL_TEXT

uname -a

# Use local build directory instead of project root
TARGET_BUILD_FOLDER="build"

# Create build directory in current working directory
mkdir -p "$TARGET_BUILD_FOLDER/python_demos" || { 
    RED_TEXT
    echo "Failed to create build directory at $TARGET_BUILD_FOLDER/python_demos"
    exit 1 
}

# Get absolute path to script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT/src/host/libpixyusb2_examples/python_demos" || { 
    RED_TEXT
    echo "Failed to enter source directory: $PROJECT_ROOT/src/host/libpixyusb2_examples/python_demos"
    echo "Please ensure the Pixy2 repository is properly cloned with all subdirectories"
    exit 1
}

# Clean previous build
rm -f _pixy*.so pixy.py pixy_wrap.cpp

# Generate SWIG wrapper
swig -c++ -python pixy.i || { RED_TEXT; echo "SWIG failed"; exit 1; }

# Build extension module
python swig.dat build_ext --inplace -D__LINUX__ > build.log 2>&1 || { 
    RED_TEXT
    echo "Build failed - see build.log"
    cat build.log
    exit 1
}

# Copy only the necessary files
cp pixy.py _pixy*.so "$TARGET_BUILD_FOLDER/python_demos" || { RED_TEXT; echo "Failed to copy build artifacts"; exit 1; }

# Verify build output
files=("$TARGET_BUILD_FOLDER/python_demos/_pixy*.so")
if (( ${#files[@]} )); then
  GREEN_TEXT
  echo "SUCCESS: Built ${#files[@]} .so files"
  echo "Build artifacts:"
  ls -l ../../../../build/python_demos
else
  RED_TEXT
  echo "FAILURE: No .so files were built"
  echo "Build output:"
  cat build.log
  exit 1
fi

NORMAL_TEXT

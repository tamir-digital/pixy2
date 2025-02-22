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

TARGET_BUILD_FOLDER=../build

mkdir -p $TARGET_BUILD_FOLDER/python_demos || { echo "Failed to create build directory"; exit 1; }

cd ../src/host/libpixyusb2_examples/python_demos || { echo "Failed to enter source directory"; exit 1; }

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
cp pixy.py _pixy*.so ../../../../build/python_demos || { RED_TEXT; echo "Failed to copy build artifacts"; exit 1; }

# Verify build output
files=(../../../../build/python_demos/_pixy*.so)
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

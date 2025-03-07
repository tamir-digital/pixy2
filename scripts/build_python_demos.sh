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
uname -a

# Get absolute path to script directory first
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Use local build directory instead of project root
TARGET_BUILD_FOLDER="../build"  # Original location relative to scripts dir

# Create build directory in current working directory with full path
BUILD_DIR="$SCRIPT_DIR/../../build/python_demos"  # Proper path from scripts -> project root
mkdir -p "$BUILD_DIR" || { 
    RED_TEXT
    echo "Failed to create build directory at $BUILD_DIR"
    exit 1 
}
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
cp pixy.py _pixy*.so *.py "$BUILD_DIR" || { RED_TEXT; echo "Failed to copy build artifacts to $BUILD_DIR"; exit 1; }  # Copy demo scripts too

# Verify build output
files=("$BUILD_DIR/_pixy*.so")
if (( ${#files[@]} )); then
  GREEN_TEXT
  echo "SUCCESS: Built ${#files[@]} .so files"
  echo "Build artifacts:"
  ls -l "$BUILD_DIR"
else
  RED_TEXT
  echo "FAILURE: No .so files were built"
  echo "Build output:"
  cat build.log
  exit 1
fi

NORMAL_TEXT

# Create a test runner script
TEST_RUNNER="$BUILD_DIR/run_demos.sh"
echo "#!/bin/bash" > "$TEST_RUNNER"
echo "export PYTHONPATH=\"$BUILD_DIR\"" >> "$TEST_RUNNER"
echo "echo 'Available demos:'" >> "$TEST_RUNNER"
echo "ls $BUILD_DIR/*.py | grep -v pixy.py" >> "$TEST_RUNNER"
echo "echo 'To run a demo, use: python3 -m demo_name (without .py)'" >> "$TEST_RUNNER"
chmod +x "$TEST_RUNNER"

GREEN_TEXT
echo "Created test runner: $TEST_RUNNER"
NORMAL_TEXT

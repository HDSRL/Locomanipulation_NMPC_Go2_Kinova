#!/bin/bash

# Navigate to the SRB_NMPC_Casadi_Formulation/build directory, create it if it does not exist
if [ ! -d "./build" ]; then
  mkdir -p ./build
fi

if [ ! -d "./SRB_NMPC_Casadi_Formulation/build" ]; then
  mkdir -p ./SRB_NMPC_Casadi_Formulation/build
fi
cd ./SRB_NMPC_Casadi_Formulation/build
rm -rf *
cmake ..
# Run make with 16 parallel jobs
make -j16

# Execute the nmpccodegen executable
./a1_kinova_nmpccodegen

# Copy the specified files to the target directory
cp a1_kinova_nlp_code.so ../../build/
cp a1_kinova_nlp_code_f.so ../../build/
cd ../..
# Print a message indicating the script has completed
echo "Build and file copy completed successfully."

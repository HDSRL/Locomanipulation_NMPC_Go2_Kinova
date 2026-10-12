# clear && cd build &&
if [ ! -d "./build" ]; then
  mkdir -p ./build
  mkdir -p ./build/tmp
fi
cd ./build

if [ ! -d "./tmp" ]; then
  mkdir -p ./tmp
fi

# rm -rf *
cmake ..

make -j16 && ./A1_Kinova_Run_Sim ../params/LL_w_CLF.txt ../params/Walking_params.txt ../params/A1_Kinova_NMPC_params.txt ; 
# cd build && make && sudo ./Test
cd ..

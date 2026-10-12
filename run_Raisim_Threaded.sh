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

make -j16 && ./Run_Sim_Debug ../params/LL_w_CLF_A1only.txt ../params/Walking_params.txt ../params/A1_Only_NMPC_params.txt ; 
# cd build && make && sudo ./Test
cd ..

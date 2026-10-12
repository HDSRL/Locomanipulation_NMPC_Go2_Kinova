
cd build
# rm ./tmp/*_append.txt 
make -j16 && sudo ./Run_Exp_Debug ../params/LL_w_CLF_A1only.txt ../params/Walking_params.txt ../params/A1_Only_NMPC_params.txt
cd ..

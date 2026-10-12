//
// Authror: Randy Fawcett on 09/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "Parameters.hpp"

Parameters::Parameters(int argc, char *argv[]){
    int loadMPC = 0; 
    int loadLL = 0;
    int loadWalking = 0;
    int loadA1_Kinova_NMPC = 0;
    int loadA1_NMPC = 0;
    for(int i=1; i<argc; i++){
        std::string path = argv[i];
        std::string filenameOnly = path.substr(path.find_last_of("/\\") + 1);
        if(filenameOnly.find("LL")!=std::string::npos){
            loadLL = i;
        }
        if(filenameOnly.find("MPC")!=std::string::npos){
            loadMPC = i;
        }
        if(filenameOnly.find("Walking")!=std::string::npos){
            loadWalking = i;
        }
        if (filenameOnly.find("A1_Kinova_NMPC")!=std::string::npos){
            loadA1_Kinova_NMPC = i;
        }
        if (filenameOnly.find("A1_Only_NMPC")!=std::string::npos){
            loadA1_NMPC = i;
        }
        
        
    }

    if(loadMPC>0){
        std::cout << "Loading: "<< argv[loadMPC] << "..." <<std::endl;
        loadMPCParams(argv[loadMPC]);
    } else {
        // Default MPC Parameters
        mpc_params.mu_MPC = 0.7;

        mpc_params.qpx = 1e3;
        mpc_params.qpy = 1e3;
        mpc_params.qpz = 1e3;
        mpc_params.qvx = 1e3;
        mpc_params.qvy = 1e3;
        mpc_params.qvz = 1e3;
        mpc_params.qrr = 1e3;
        mpc_params.qrp = 1e3;
        mpc_params.qry = 1e3;
        mpc_params.qwr = 1e3;
        mpc_params.qwp = 1e3;
        mpc_params.qwy = 1e3;
        mpc_params.pp = 40;
        mpc_params.pv = 40;
        mpc_params.pr = 40;
        mpc_params.pw = 40;
        mpc_params.rx = 1e-2;
        mpc_params.ry = 1e-2;
        mpc_params.rz = 1e-2;
        mpc_params.updateEveryItter = 0;
        mpc_params.useMPCTraj = 0;
    }

    if(loadLL>0){
        std::cout<<"Loading: "<< argv[loadLL] << "..." <<std::endl;
        loadLowLevelParams(argv[loadLL]);
    } else {
        // Default Low Level
        ll_params.mu = 0.7;
        ll_params.kp = 700;   // proportional gain on IO linearization
        ll_params.kd = 40;    // derivative gain on IO linearization
        ll_params.useCLF = true;

        // Cost
        ll_params.tauPen = 1e0; // input weight (cost)
        ll_params.dfPen  = 1e-1; // ||F-F_d|| weight (cost)
        ll_params.auxPen = 1e6; // auxiliary var weight (cost)
        ll_params.clfPen = 1e8; // clf defect var weight (cost)
        
        // Constraints
        ll_params.auxMax = 100;  // max aux value (constraint)
        ll_params.clfEps = 0.8; // CLF convergence tuning variable
    }

    if(loadWalking>0){
        std::cout<<"Loading: "<< argv[loadWalking] << "..." <<std::endl;
        loadWalkingParams(argv[loadWalking]);
    } else {
        // Default 
        motion_params.standHeight = 0.26;
        motion_params.swingHeight = 0.06;
        motion_params.fwdSpeed = 0.0;
        motion_params.latSpeed = 0.0;
        motion_params.yawSpeed = 0.0;
        motion_params.neverStopTrot = 1;
        motion_params.extCmd = 0;
    }

    if (loadA1_Kinova_NMPC>0){
        std::cout<<"Loading: "<< argv[loadA1_Kinova_NMPC] << "..." <<std::endl;
        loadA1_Kinova_NMPCParams(argv[loadA1_Kinova_NMPC]);
    } else      
    {
        // Default A1_Kinova_NMPC
        a1_kinova_nmpc_params.Qpx = 1e5;
        a1_kinova_nmpc_params.Qpy = 1e5;
        a1_kinova_nmpc_params.Qpz = 1e5;
        a1_kinova_nmpc_params.Qvx = 1e5;
        a1_kinova_nmpc_params.Qvy = 1e5;
        a1_kinova_nmpc_params.Qvz = 1e5;
        a1_kinova_nmpc_params.Qrr = 1e3;
        a1_kinova_nmpc_params.Qrp = 1e3;
        a1_kinova_nmpc_params.Qry = 1e3;
        a1_kinova_nmpc_params.Qwr = 5e3;
        a1_kinova_nmpc_params.Qwp = 5e3;
        a1_kinova_nmpc_params.Qwy = 5e3;
        a1_kinova_nmpc_params.Pp = 1e1;
        a1_kinova_nmpc_params.Pv = 1e1;
        a1_kinova_nmpc_params.Pr = 1e1;
        a1_kinova_nmpc_params.Pw = 1e1;
        a1_kinova_nmpc_params.RFR = 1e0;
        a1_kinova_nmpc_params.RFL = 1e0;
        a1_kinova_nmpc_params.RRR = 1e0;
        a1_kinova_nmpc_params.RRL = 1e0;
        a1_kinova_nmpc_params.Qp_abx = 0;
        a1_kinova_nmpc_params.Qp_aby = 0;
        a1_kinova_nmpc_params.Qp_abz = 0;
        a1_kinova_nmpc_params.Qr_abx = 0;
        a1_kinova_nmpc_params.Qr_aby = 0;
        a1_kinova_nmpc_params.Qr_abz = 0;
        a1_kinova_nmpc_params.Qq_s1 = 8e5;
        a1_kinova_nmpc_params.Qq_s2 = 8e7;  
        a1_kinova_nmpc_params.Qq_s3 = 8e5;
        a1_kinova_nmpc_params.Qq_s4 = 3e3;
        a1_kinova_nmpc_params.Qdp_abx = 0;
        a1_kinova_nmpc_params.Qdp_aby = 0;
        a1_kinova_nmpc_params.Qdp_abz = 0;
        a1_kinova_nmpc_params.Qomega_abx = 0;
        a1_kinova_nmpc_params.Qomega_aby = 0;
        a1_kinova_nmpc_params.Qomega_abz = 0;
        a1_kinova_nmpc_params.Qdq_s1 = 8e3;
        a1_kinova_nmpc_params.Qdq_s2 = 8e5;
        a1_kinova_nmpc_params.Qdq_s3 = 8e3;
        a1_kinova_nmpc_params.Qdq_s4 = 8e1;
        a1_kinova_nmpc_params.Ra1 = 1e0;
        a1_kinova_nmpc_params.Ra2 = 1e0;
        a1_kinova_nmpc_params.Ra3 = 1e0;
        a1_kinova_nmpc_params.Ra4 = 1e0;
        a1_kinova_nmpc_params.R_f1 = 1e1;
        a1_kinova_nmpc_params.R_f2 = 1e1;
        a1_kinova_nmpc_params.R_f3 = 1e1;
        a1_kinova_nmpc_params.R_tau1 = 5e0;
        a1_kinova_nmpc_params.R_tau2 = 5e0;
        a1_kinova_nmpc_params.R_tau3 = 5e0;
        a1_kinova_nmpc_params.Pp_ab = 1e1;
        a1_kinova_nmpc_params.Pr_ab = 1e1;
        a1_kinova_nmpc_params.Pq_s = 1e1;
        a1_kinova_nmpc_params.Pdp_ab = 1e1;
        a1_kinova_nmpc_params.Pomega_ab = 1e1;
        a1_kinova_nmpc_params.Pq_s = 1e1;
        a1_kinova_nmpc_params.solver_iter = 10;
 

    }

    if (loadA1_NMPC>0){
        std::cout<<"Loading: "<< argv[loadA1_NMPC] << "..." <<std::endl;
        loadA1_NMPCParams(argv[loadA1_NMPC]);
    } else      
    {
        // Default A1_NMPC
        a1_nmpc_params.Qpx = 1e6;
        a1_nmpc_params.Qpy = 1e6;
        a1_nmpc_params.Qpz = 8e6;
        a1_nmpc_params.Qvx = 1e5;
        a1_nmpc_params.Qvy = 1e5;
        a1_nmpc_params.Qvz = 1e5;
        a1_nmpc_params.Qrr = 1e6;
        a1_nmpc_params.Qrp = 1e6;
        a1_nmpc_params.Qry = 1e6;
        a1_nmpc_params.Qwr = 1e4;
        a1_nmpc_params.Qwp = 1e4;
        a1_nmpc_params.Qwy = 1e4;
        a1_nmpc_params.Pp = 1e1;
        a1_nmpc_params.Pv = 1e1;
        a1_nmpc_params.Pr = 1e1;
        a1_nmpc_params.Pw = 1e1;
        a1_nmpc_params.RFR = 1e0;
        a1_nmpc_params.RFL = 1e0;
        a1_nmpc_params.RRR = 1e0;
        a1_nmpc_params.RRL = 1e0;    
    
    }
    
    
    
};

Parameters::~Parameters(){};

void Parameters::loadLowLevelParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    
    int j;
    j = fscanf(fid, "%lf\n", &ll_params.mu);
    j = fscanf(fid, "%lf\n", &ll_params.kp);
    j = fscanf(fid, "%lf\n", &ll_params.kd);
    j = fscanf(fid, "%d\n",  &ll_params.useCLF);
    j = fscanf(fid, "%lf\n", &ll_params.tauPen);
    j = fscanf(fid, "%lf\n", &ll_params.dfPen);
    j = fscanf(fid, "%lf\n", &ll_params.auxPen);
    j = fscanf(fid, "%lf\n", &ll_params.clfPen);
    j = fscanf(fid, "%lf\n", &ll_params.auxMax);
    j = fscanf(fid, "%lf\n", &ll_params.clfEps);
    j = fclose(fid);
};

void Parameters::loadMPCParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    int j;
    j = fscanf(fid, "%lf", &mpc_params.mu_MPC);
    j = fscanf(fid, "%lf", &mpc_params.qpx);
    j = fscanf(fid, "%lf", &mpc_params.qpy);
    j = fscanf(fid, "%lf", &mpc_params.qpz);
    j = fscanf(fid, "%lf", &mpc_params.qvx);
    j = fscanf(fid, "%lf", &mpc_params.qvy);
    j = fscanf(fid, "%lf", &mpc_params.qvz);
    j = fscanf(fid, "%lf", &mpc_params.qrr);
    j = fscanf(fid, "%lf", &mpc_params.qrp);
    j = fscanf(fid, "%lf", &mpc_params.qry);
    j = fscanf(fid, "%lf", &mpc_params.qwr);
    j = fscanf(fid, "%lf", &mpc_params.qwp);
    j = fscanf(fid, "%lf", &mpc_params.qwy);
    j = fscanf(fid, "%lf", &mpc_params.pp);
    j = fscanf(fid, "%lf", &mpc_params.pv);
    j = fscanf(fid, "%lf", &mpc_params.pr);
    j = fscanf(fid, "%lf", &mpc_params.pw);
    j = fscanf(fid, "%lf", &mpc_params.rx);
    j = fscanf(fid, "%lf", &mpc_params.ry);
    j = fscanf(fid, "%lf", &mpc_params.rz);
    j = fscanf(fid, "%lf", &mpc_params.updateEveryItter);
    j = fscanf(fid, "%lf", &mpc_params.useMPCTraj);
    j = fclose(fid);
};

void Parameters::loadWalkingParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    
    int j;
    j = fscanf(fid, "%lf\n", &motion_params.standHeight);
    j = fscanf(fid, "%lf\n", &motion_params.swingHeight);
    j = fscanf(fid, "%lf\n", &motion_params.fwdSpeed);
    j = fscanf(fid, "%lf\n", &motion_params.latSpeed);
    j = fscanf(fid, "%lf\n", &motion_params.yawSpeed);
    j = fscanf(fid, "%lf\n", &motion_params.neverStopTrot);
    j = fscanf(fid, "%lf\n", &motion_params.extCmd);
    j = fclose(fid);

}


void Parameters::loadA1_Kinova_NMPCParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    
    int j;
    //Scan comma separated Qpx,Qpy,Qpz from first row of text file
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qpx, &a1_kinova_nmpc_params.Qpy, &a1_kinova_nmpc_params.Qpz);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qvx, &a1_kinova_nmpc_params.Qvy, &a1_kinova_nmpc_params.Qvz);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qrr, &a1_kinova_nmpc_params.Qrp, &a1_kinova_nmpc_params.Qry);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qwr, &a1_kinova_nmpc_params.Qwp, &a1_kinova_nmpc_params.Qwy);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Pp, &a1_kinova_nmpc_params.Pv, &a1_kinova_nmpc_params.Pr, &a1_kinova_nmpc_params.Pw);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_kinova_nmpc_params.RFR, &a1_kinova_nmpc_params.RFL, &a1_kinova_nmpc_params.RRR, &a1_kinova_nmpc_params.RRL);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qp_abx, &a1_kinova_nmpc_params.Qp_aby, &a1_kinova_nmpc_params.Qp_abz);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qr_abx, &a1_kinova_nmpc_params.Qr_aby, &a1_kinova_nmpc_params.Qr_abz);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qq_s1, &a1_kinova_nmpc_params.Qq_s2, &a1_kinova_nmpc_params.Qq_s3, &a1_kinova_nmpc_params.Qq_s4);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qdp_abx, &a1_kinova_nmpc_params.Qdp_aby, &a1_kinova_nmpc_params.Qdp_abz);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qomega_abx, &a1_kinova_nmpc_params.Qomega_aby, &a1_kinova_nmpc_params.Qomega_abz);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Qdq_s1, &a1_kinova_nmpc_params.Qdq_s2, &a1_kinova_nmpc_params.Qdq_s3, &a1_kinova_nmpc_params.Qdq_s4);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Ra1, &a1_kinova_nmpc_params.Ra2, &a1_kinova_nmpc_params.Ra3, &a1_kinova_nmpc_params.Ra4);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.R_f1, &a1_kinova_nmpc_params.R_f2, &a1_kinova_nmpc_params.R_f3);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_kinova_nmpc_params.R_tau1, &a1_kinova_nmpc_params.R_tau2, &a1_kinova_nmpc_params.R_tau3);
    j = fscanf(fid, "%lf,%lf,%lf,%lf,%lf,%lf \n", &a1_kinova_nmpc_params.Pp_ab, &a1_kinova_nmpc_params.Pr_ab, &a1_kinova_nmpc_params.Pq_s, &a1_kinova_nmpc_params.Pdp_ab, &a1_kinova_nmpc_params.Pomega_ab, &a1_kinova_nmpc_params.Pdq_s);
    j = fscanf(fid, "%d\n", &a1_kinova_nmpc_params.solver_iter);
    j = fclose(fid);
};

void Parameters::loadA1_NMPCParams(char* filename){
    FILE *fid;
    errno = 0;
    fid = fopen(filename,"r");
    if(fid==NULL){
        std::cout<<strerror(errno)<<std::endl;
    }
    std::cout<<"I read from here!"<<std::endl;
    
    int j;
    //Scan comma separated Qpx,Qpy,Qpz from first row of text file
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_nmpc_params.Qpx, &a1_nmpc_params.Qpy, &a1_nmpc_params.Qpz);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_nmpc_params.Qvx, &a1_nmpc_params.Qvy, &a1_nmpc_params.Qvz);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_nmpc_params.Qrr, &a1_nmpc_params.Qrp, &a1_nmpc_params.Qry);
    j = fscanf(fid, "%lf,%lf,%lf \n", &a1_nmpc_params.Qwr, &a1_nmpc_params.Qwp, &a1_nmpc_params.Qwy);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_nmpc_params.Pp, &a1_nmpc_params.Pv, &a1_nmpc_params.Pr, &a1_nmpc_params.Pw);
    j = fscanf(fid, "%lf,%lf,%lf,%lf \n", &a1_nmpc_params.RFR, &a1_nmpc_params.RFL, &a1_nmpc_params.RRR, &a1_nmpc_params.RRL);
    j = fclose(fid);
};
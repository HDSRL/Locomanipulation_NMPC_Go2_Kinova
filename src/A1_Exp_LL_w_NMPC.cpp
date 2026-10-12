//
// Authror: Randy Fawcett on 12/2021.
//
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//
#include "LocoWrapper.hpp"
#include "nmpc.hpp"
#include "shared_structs.hpp"

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "timer.h"
#include "Transforms.hpp"
#include "Filters.h"
#include "OtherUtils.hpp"
#include <yaml-cpp/yaml.h>
#include "stdio.h"
#include "fstream"

#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()

using namespace UNITREE_LEGGED_SDK;
bool turn_off_both = false;
bool soft_fall_both = false;
bool start_both = false;
sharedData agentData;


class ExternalComm {
public:
	 	// ExternalComm() : udpComp0(8082, "192.168.123.10", 8007, sizeof(LowCmd), sizeof(LowState)){
		ExternalComm() : udpComp0(LOWLEVEL){
		fid = fopen("/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/stateData_0003.csv", "w");
		
		double ad[3] = {1.0, -1.47548044359265, 0.58691950806119};
		double bd[3] = {0.02785976611714, 0.05571953223427, 0.02785976611714};
		populate_filter_d(jointfilter,ad,bd,3,12);

		// 0.75 Hz
		float af[3] = {1.00000000,-1.99333570,0.99335783};
		float bf[3] = {0.00000553,0.00001107,0.00000553};
		populate_filter_f(remotefilter, af, bf, 3, 2);

		// 2 Hz 
		float aa[3] = {1.00000000,-1.98222893,0.98238545};
		float ba[3] = {0.00003913,0.00007826,0.00003913};
		populate_filter_f(angfilter, aa, ba, 3, 2);

		
	}

	virtual ~ExternalComm(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);
	}

	// for first agent
	void Calc();
	void HighLevel();

	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<NMPC> mpc_obj;
	UDP udpComp0;
	FILE *fid;

	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));

	LowState state = {0};
	LowCmd cmd = {0};
	xRockerBtnDataStruct remote;

	int stop = 0;
	float vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

	float dt = 0.001f;

};

void ExternalComm::HighLevel(){
	timer tset;
	// printf("[MPC] Time: %f\n",1000*toc(&tset));

	static sharedData HLData0;

	// std::cout << "[MPC] Updating Data set \n";
	// Get the data for the high level
	updateData(GET_DATA, HL_DATA, &HLData0, 0);
	std::cout << "[MP] Motiontime: " << HLData0.motiontime << std::endl;
	mpc_obj->updateState(HLData0.q, 
                         HLData0.dq,
                         HLData0.ind,
                         HLData0.toePos,
                         HLData0.last_state1,
                         HLData0.domain,
                         HLData0.motiontime);
	
		HLData0.runMPC = 0;
		mpc_obj->run_NMPC();


		HLData0.MPC_data_available = 1;
		HLData0.alpha_COM = mpc_obj->get_alphaCOM();
		HLData0.alpha_GRF = mpc_obj->get_alphaGRF();
		HLData0.MPC_sol_ = mpc_obj->get_MPCsol();
		HLData0.MPC_sol_GRF_ = mpc_obj->get_MPCsolGRF();
		
		HLData0.resetRun = -1;

	updateData(SET_DATA, HL_DATA, &HLData0, 0);

}

void ExternalComm::Calc() {

	static sharedData LLData0;
	// std::cout << "Updating Data\n";
	updateData(GET_DATA, LL_DATA, &LLData0, 0);
	// std::cout << "Setting up UDP connection \n";
	udpComp0.Recv();
	udpComp0.GetRecv(state);
	motiontime += 1;
	// std::cout << "Setup done UDP connection \n";

	int standtime = 10000;
	int starttime = 8000;

	// std::cout << "Reading Wireless Stuff \n";
	// ===================================================== //
	// ============== Wireless Remote Stuff  =============== //
	// ===================================================== //
	memcpy(&remote, state.wirelessRemote, 40);
	if((int)remote.btn.components.B!=0 && (int)remote.btn.components.L2!=0){
		stop = 1;
		cmd.reserve = 0;
	}
	if((int)remote.btn.components.A!=0 && (int)remote.btn.components.L2!=0){
		softFall = true;
		// soft_fall_both = true;
	}
	if( ((int)remote.btn.components.A!=0) && ((int)remote.btn.components.L2 == 0) ){
		// std::cout << "A is pressed!" <<std::endl;
		beginCommand = 1;
		cmd.reserve = 1;
		// start_both = true;
	}
	// std::cout << "Setting up first command \n";
	vel[0] = 0.75f*remote.ly; // x vel
	vel[1] = -0.4f*remote.rx; // y vel
	ang[0] = 20.0f*3.14f/180.0f*remote.ry; // yaw vel
	ang[1] = -2.0f*remote.lx; // pitch pos
	discrete_butter_f(remotefilter,vel);
	discrete_butter_f(angfilter,ang);
	pose[4] = ang[0]; // set filtered pose
	vel[2] = ang[1];  // set filtered ang vel
	// std::cout << "Updating State \n";
	// ===================================================== //
	// ================= Update the State ================== //
	// ===================================================== //
	for (int i=0; i<12; ++i){
		q[i+6] = state.motorState[i].q;
		dq[i+6] = state.motorState[i].dq;
	}
	discrete_butter_d(jointfilter,&dq[6]);

	q[3] = state.imu.rpy[0]; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	dq[3] = state.imu.gyroscope[0]; dq[4] = state.imu.gyroscope[1]; dq[5] = state.imu.gyroscope[2];

	// std::cout << "[LL] RUNNING CALC (motiontime:) " << std::to_string(motiontime) << " Height " << q[2] <<"\n";



	quat_to_XYZ(state.imu.quaternion[0],state.imu.quaternion[1],state.imu.quaternion[2],state.imu.quaternion[3],
	            q[3],q[4],q[5]);
	//Offset for Halo 
	q[3]+= -3.0*MY_PI/180.0;
	q[4] += 1.5*MY_PI/180.0;
	q[5] += 3.0*MY_PI/180.0;
	/* //Offset for Willow
	q[3]+=  0*MY_PI/180.0;
	q[4] += 0*MY_PI/180.0;
	q[5] += 0*MY_PI/180.0;
	//dq[5] += 5.0*MY_PI/180.0; */

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(state.imu.quaternion,R);
	rotMat = R.data();

	// Kinematic Estimation
	int footForce[4];
	footForce[0] = state.footForce[0]; footForce[1] = state.footForce[1];
	footForce[2] = state.footForce[2]; footForce[3] = state.footForce[3];
	static int contactIndex[4] = {1,1,1,1};
	kinEst0(footForce,contactIndex,q,dq,R); // Defined in OtherUtils.hpp
	std::cout << "[LL] Motiontime: " << motiontime << std::endl;
	// ===================================================== //
	// ============= Quad Initialization Time ============== //
	// ===================================================== //
	if ( (motiontime < starttime) ){
		for(int i=0; i<12; ++i){
			cmd.motorCmd[i].dq = 0.0;
			cmd.motorCmd[i].Kp = 0.0f;
			cmd.motorCmd[i].Kd = 0.0f;
			cmd.motorCmd[i].tau = 0.0f;
		}
	}
	// ===================================================== //
	// ================== LL Controller ==================== //
	// ===================================================== //
	// std::cout << "Controller Executing \n";
	static long int endStand = standtime;
	if ( motiontime >= starttime ){
		if (beginCommand & !softFall){
			double phaseVar;
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				endStand = motiontime+standtime;
				loco_obj->initStandVars(com,standtime);
				setup = false;
			}
			
			beginPose = (motiontime<endStand) ? 0 : 1;
			if (beginPose == 0){
				// std::cout << "\n[LL] In Standing Mode!\n";
				// loco_obj->set_MPC_DATA( LLData0.alpha_COM,
                //                         LLData0.MPC_sol_, 
                //                         LLData0.MPC_data_available);
				loco_obj->set_MPC_DATA_W_GRF(	LLData0.alpha_COM,
												LLData0.MPC_sol_, 
												LLData0.alpha_GRF, 
												LLData0.MPC_sol_GRF_,
												LLData0.MPC_data_available);
				// Calculate the torque
				// loco_obj->calcTau(q, dq, rotMat, footForce, STAND, motiontime, &(LLData0.runMPC));		//runMPC pointer not passed in the last argument
				// loco_obj->calcZOHTauWithGRF(q,dq,rotMat,footForce,STAND_MPC,motiontime, &(LLData0.runMPC));
				loco_obj->calcZOHTauWithGRF(q,dq,rotMat,footForce,STAND,motiontime, &(LLData0.runMPC));
				// loco_obj->calcTauLLonly(q, dq, rotMat, footForce, STAND, motiontime,&(LLData0.runMPC));
			}
			else if (beginPose == 1 && motiontime>=endStand)
			{
				// std::cout << "\n[LL] In TROT Mode!\n";
				// loco_obj->set_MPC_DATA( LLData0.alpha_COM, 
                //                         LLData0.MPC_sol_, 
                //                         LLData0.MPC_data_available);
				

				loco_obj->set_MPC_DATA_W_GRF(	LLData0.alpha_COM,
												LLData0.MPC_sol_, 
												LLData0.alpha_GRF, 
												LLData0.MPC_sol_GRF_,
												LLData0.MPC_data_available);
				// Calculate the torque
				// loco_obj->calcTau(q, dq, rotMat, footForce, TROT, motiontime, &(LLData0.runMPC));		//runMPC pointer not passed in the last argument
				loco_obj->calcZOHTauWithGRF(q,dq,rotMat,footForce,TROT,motiontime, &(LLData0.runMPC));
				// loco_obj->calcTauLLonly(q, dq, rotMat, footForce, TROT, motiontime,&(LLData0.runMPC));


				const int* contactMat = loco_obj->getConDes();
				
				memcpy(contactIndex,contactMat,4*sizeof(int));
			}
			LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
			LLData0.toePos = loco_obj->get_toePos();
            LLData0.domain = loco_obj->getDomain();
            LLData0.domain_change = loco_obj->hasDomainChanged();
			memcpy(LLData0.ind,contactIndex,4*sizeof(int));

			if(LLData0.resetRun == -1){
				LLData0.resetRun = -2;
			}

			// Set the command
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = loco_obj->ll->tau[i + 6];
				cmd.motorCmd[i].q  = loco_obj->ll->q(i + 6);
				cmd.motorCmd[i].dq = loco_obj->ll->dq(i + 6);

				cmd.motorCmd[i].Kp = 10;
				cmd.motorCmd[i].Kd = 8;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
			//							cmd.motorCmd[i].tau = 0;
							cmd.motorCmd[3*i].Kp = 20;
							cmd.motorCmd[3*i+1].Kp = 20;
							cmd.motorCmd[3*i+2].Kp = 20;

							cmd.motorCmd[3*i].Kd = 1;
							cmd.motorCmd[3*i+1].Kd = 1;
							cmd.motorCmd[3*i+2].Kd = 1;  
						}
					}	
				}
			}

			// Saturate the command
			float hr_max = 10.0f, hr_min = -10.0f;
			float hp_max = 30.0f, hp_min = -30.0f;
			float kn_max = 33.0f, kn_min = -33.0f;
			for (int i = 0; i < 4; i++){
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau > hr_max) ? hr_max : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 0].tau = (cmd.motorCmd[3 * i + 0].tau < hr_min) ? hr_min : cmd.motorCmd[3 * i + 0].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau > hp_max) ? hp_max : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 1].tau = (cmd.motorCmd[3 * i + 1].tau < hp_min) ? hp_min : cmd.motorCmd[3 * i + 1].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau > kn_max) ? kn_max : cmd.motorCmd[3 * i + 2].tau;
				cmd.motorCmd[3 * i + 2].tau = (cmd.motorCmd[3 * i + 2].tau < kn_min) ? kn_min : cmd.motorCmd[3 * i + 2].tau;
			}

		}
		else if ( softFall ) 
		{
			for (int i = 0; i < 12; ++i){
				cmd.motorCmd[i].tau = 0.0;
				cmd.motorCmd[i].q   = 0.0;
				cmd.motorCmd[i].dq  = 0.0;

				cmd.motorCmd[i].Kp = 0;
				cmd.motorCmd[i].Kd = ( ( (i+1)%3 ) == 0 ) ? 6 : 3;
			}
		}
		else if ( (!beginCommand) && (!softFall) )
		{
			static bool printedA = false;
			if(!printedA){
				printf("\nPress A to continue\n");
				printedA = true;
			}
		}
	}

	// std::cout << std::endl;
	// for (int i = 0; i < 4; i++){
	// 	std::cout << "Torques Leg " << i << ": " << cmd.motorCmd[3 * i + 0].tau<< " " << cmd.motorCmd[3 * i + 1].tau<< " " << cmd.motorCmd[3 * i + 2].tau<<std::endl;
	// }
	// std::cout << std::endl;
	// LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
	// LLData0.toePos = loco_obj->get_toePos();
	// LLData0.domain = loco_obj->getDomain();
	// LLData0.domain_change = loco_obj->hasDomainChanged();
	// memcpy(LLData0.ind,contactIndex,4*sizeof(int));

	// if(LLData0.resetRun == -1){
	// 	LLData0.resetRun = -2;
	// }
	const int* contactMat = loco_obj->getConDes();
	memcpy(LLData0.q,q,18*sizeof(double));
    memcpy(LLData0.dq,dq,18*sizeof(double));
	memcpy(LLData0.ind, contactMat,4*sizeof(int));
	LLData0.motiontime = motiontime;
	//	printf("%f\n",1000*toc(&tset));
	updateData(SET_DATA, LL_DATA, &LLData0, 0);

	udpComp0.SetSend(cmd);
	udpComp0.Send();
}


int main(int argc, char *argv[])
{
	InitEnvironment();
	ExternalComm extComm;

	Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart;

	// Pstart << 0.0, 0.0, 0, -1.5;//, 1.0, -2, 1, -3;	//2.0, -2.0
	
    Pstart << 0.0, 0.0; //, 1.0, -2, 1, -3;	//2.0, -2.0

	extComm.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv,false)); //isSim = False
	extComm.mpc_obj  = std::unique_ptr<NMPC>(new NMPC(argc,argv));

	// Agent ID 0
	extComm.loco_obj->setAgentID(0);
    extComm.loco_obj->setPstart(Pstart);

    extComm.mpc_obj->init();
	extComm.mpc_obj->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
    extComm.mpc_obj->setPstart(Pstart);
    // extComm.mpc_obj->generateReferenceTrajectory(); //Not needed

	data0.alpha_COM.setZero(12,HORIZON);
	data0.alpha_GRF.setZero(12,HORIZON);
		
	// LoopFunc loop_calc("calc_loop", extComm.dt,1, boost::bind(&ExternalComm::Calc, &extComm));
	LoopFunc loop_calc("calc_loop", extComm.dt,1, boost::bind(&ExternalComm::Calc, &extComm));
	LoopFunc loop_mpc("mpc_loop", 0.010001f,2, boost::bind(&ExternalComm::HighLevel, &extComm));


	loop_mpc.start();
	sleep(1);
	loop_calc.start();
	sleep(1);

	extComm.udpComp0.InitCmdData(extComm.cmd);
	
	while ( (extComm.stop==0) ){
		sleep(0.1);
	};
	fclose(extComm.fid);
	return 0;
}
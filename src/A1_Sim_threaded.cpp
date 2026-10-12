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
#include "raisim/OgreVis.hpp"
#include "randyImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "raisim/interfaces.hpp"
#include "helper.hpp"
#include "helper2.hpp"
#include "MPC_dist.hpp"
#include "nmpc.hpp"
#include "OtherUtils.hpp"
#include "timer.h"
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time()
#include <fstream>
#include <iostream>

#include "Transforms.hpp"
#include "Filters.h"
#include "OtherUtils.hpp"
#include <yaml-cpp/yaml.h>
#include "stdio.h"
using namespace UNITREE_LEGGED_SDK;




class ExternalComm {
public:
	 	// ExternalComm() : udpComp0(8082, "192.168.123.10", 8007, sizeof(LowCmd), sizeof(LowState)){
		ExternalComm(){
		// fid = fopen("/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/stateData_0003.csv", "w");
		
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

			StandDuration = 10000;
			SettlingTime = 8000;

			/********************************************RASIM INIT********************************************** */
			int simcounter = 0;
			const int NUMBER_OF_SIMS = 1;

			const float threshold = 0.4;

			bool shared_data_backed_up = 0;

			vis = raisim::OgreVis::get();
			
		}
	
		

	virtual ~ExternalComm(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);

		mpc_obj->logData();

	// outFile << failingDistance0 << "," << failingDistance1 << std::endl;

		loco_obj.reset();
		// delete loco_obj2;
		// delete MPC_Agent0;
		// delete MPC_Agent1;
	}


	//support functions
	void setupCallback();
	void plotGRFs(std::map<std::string, raisim::VisualObject>* objList, const std::vector<double>& GRFs, const std::vector<double>& pf, const std::vector<double>& contact);
	std::string getCurrentTimestamp();

	// main thread execution functions
	void Calc();
	void HighLevel();
	void SimExec();
	




	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<NMPC> mpc_obj;
	// FILE *fid;


	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};
	double rotMatrixDouble[9] = {0.0};
	int force[4] = {0};

	int StandDuration;
	int SettlingTime;

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));

	int stop = 0;
	float vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

	float dt = 0.001f;
 // Some parameters in controller0
	size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 8*ctrlHz;                   // Settling down
    size_t duration = 10*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern
    timer tset;

    double *tau;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    int conIndDes[4] = {1};

	//Raisim stuff
	bool setup_raisim = true;
	raisim::World world;
	raisim::OgreVis *vis;
	raisim::TerrainProperties terrainProperties;
	raisim::HeightMap *ground;
	std::map<std::string, raisim::VisualObject>* list = nullptr;
	std::vector<raisim::ArticulatedSystem*> A1;
	long simcounter = 0;
	bool added = false;
	std::string cameraview = "iso";
	bool panX = true;                // Pan view with robot during walking (X direction)
	bool panY = true;                // Pan view with robot during walking (Y direction)
	bool record = false;             // Record?
	double startTime = 0*ctrlHz;    // RecoPstartrding start time
	// double simlength = nmpc->params.sim_length/nmpc->params.dt; //60*ctrlHz;   // Sim end time
	double simlength = SIM_TIME*ctrlHz;
	double fps = 30;            
	std::string directory = "~/OneDrive/1Research_legged_locomotion/A1_Kinova_locomanipulation/raisim/";
	// std::string filename = "Payload_Inplace";
	std::string timestamp = getCurrentTimestamp();
	std::string filename = "SRB_NMPC_"+timestamp;
	std::string name = directory+filename+"_"+cameraview+".mp4";



};

void ExternalComm::setupCallback(){
	
	// vis = raisim::OgreVis::get();

    /// light
    vis->getLight()->setDiffuseColour(1, 1, 1);
    vis->getLight()->setCastShadows(false);
    Ogre::Vector3 lightdir(-3,3,-0.5); // Light shines on ROBOTS top/front/right side
    // Ogre::Vector3 lightdir(-3,-3,-0.5); // Light shines on ROBOTS top/front/left side
    lightdir.normalise();
    vis->getLightNode()->setDirection({lightdir});
    vis->setCameraSpeed(300);

    vis->addResourceDirectory(raisim::loadResource("material"));
    vis->loadMaterialFile("myMaterials.material");

    vis->addResourceDirectory(vis->getResourceDir() + "/material/skybox/violentdays");
    vis->loadMaterialFile("violentdays.material");

    /// shdow setting
    vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
    vis->getSceneManager()->setShadowTextureSettings(2048, 3);

    /// scale related settings!! Please adapt it depending on your map size
    // beyond this distance, shadow disappears
    vis->getSceneManager()->setShadowFarDistance(10);
    // size of contact points and contact forces
    vis->setContactVisObjectSize(0.03, 0.6);
    // speed of camera motion in freelook mode
    vis->getCameraMan()->setTopSpeed(5);

}

std::string ExternalComm::getCurrentTimestamp() {
    // Get the current time from the system clock
    auto now = std::chrono::system_clock::now();
    
    // Convert to a time_t (a C-style time) which represents time in seconds since the epoch
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    
    // Convert the time to a tm struct for local time
    std::tm* local_time = std::localtime(&now_time);
    
    // Use a stringstream to format the time as 'yyyy-mm-dd_HH-MM-SS'
    std::ostringstream oss;
    oss << std::put_time(local_time, "%Y-%m-%d_%H-%M-%S");
    
    // Return the formatted string
    return oss.str();
}

void ExternalComm::plotGRFs(std::map<std::string, raisim::VisualObject>* objList, const std::vector<double>& GRFs, const std::vector<double>& pf, const std::vector<double>& contact) {
    // Ensure the vectors are of the correct size
    if (GRFs.size() < 12 || pf.size() < 12 || contact.size() < 4) {
        std::cerr << "Error: Input vectors are of incorrect size." << std::endl;
        return;
    }


    for (int i = 0; i < 4; ++i) {
        raisim::Vec<3> dir;
        for (int j = 0; j < 3; ++j) {
            dir[j] = GRFs[3 * i + j];
        }

        // Normalize the direction vector if it is not a zero vector
        double norm = dir.norm();
        if (norm > 1e-6) {  // Check if the vector is non-zero to avoid division by zero
            dir /= norm;
        } else {
            dir.setZero();
        }

        // Convert direction vector to a rotation matrix that aligns the z-axis with the direction vector
        raisim::Mat<3, 3> rot;
        if (contact[i] == 1 && norm > 1e-6) {
            raisim::zaxisToRotMat(dir, rot);
        } else {
            rot.setIdentity();  // Set rotation to identity if no contact or zero norm
        }

        // Visual object key
        std::string objKey = "GRF" + std::to_string(i + 1);
        (*objList)[objKey].offset = {pf[3 * i], pf[3 * i + 1], pf[3 * i + 2]};
        (*objList)[objKey].scale = {0.2, 0.2, 0.005 * norm};  // Scaling based on the norm of the GRF vector

        // Set the rotation matrix
        (*objList)[objKey].rotationOffset = rot;
    }

}



void ExternalComm::HighLevel(){

	// timer tset;
	// auto start = std::chrono::high_resolution_clock::now();
	// printf("[MPC] Time: %f\n",1000*toc(&tset));
	// std::cout << "[MPC] RUNNING MPC (motiontime:) " << std::to_string(motiontime) << "\n";

	static sharedData HLData0;

	// Get the data for the high level
	updateData(GET_DATA, HL_DATA, &HLData0, 0);
	mpc_obj->updateState(HLData0.q_LL, 
                         HLData0.dq_LL,
                         HLData0.ind_LL,
                         HLData0.toePos,
                         HLData0.last_state1,
                         HLData0.domain,
                         motiontime);

	// Update the MPC
	if (HLData0.runMPC == 1)
	{
		HLData0.runMPC = 0;
		mpc_obj->run_NMPC();

		HLData0.MPC_data_available = 1;
		HLData0.alpha_COM = mpc_obj->get_alphaCOM();
		HLData0.alpha_GRF = mpc_obj->get_alphaGRF();
		HLData0.MPC_sol_ = mpc_obj->get_MPCsol();
		HLData0.MPC_sol_GRF_ = mpc_obj->get_MPCsolGRF();
		
		HLData0.resetRun = -1;
		std::vector<double> temp_contacts = mpc_obj->get_contacts();
		std::vector<double> temp_Pf = mpc_obj->get_Pf();
		std::vector<double> temp_GRF = mpc_obj->get_GRF();

		std::copy(temp_contacts.begin(), temp_contacts.end(), HLData0.HL_contacts);
		std::copy(temp_Pf.begin(), temp_Pf.end(), HLData0.HL_feet_vec);
		std::copy(temp_GRF.begin(), temp_GRF.end(), HLData0.HL_GRF);
		
		// mpc_obj->logData();
	}

	updateData(SET_DATA, HL_DATA, &HLData0, 0);

	// auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// std::cout << "Time taken by function: "
	// 	 << duration.count() << " milliseconds" << std::endl;

}

void ExternalComm::Calc() {

	// std::cout << "[LL] RUNNING CALC (motiontime:) " << std::to_string(motiontime) << "\n";
	// auto start = std::chrono::high_resolution_clock::now();
	static sharedData LLData0;

	// Getting MPC outputs
	updateData(GET_DATA, LL_DATA, &LLData0, 0);
	// Taking feedback
	motiontime += 1;
	const int* contactMat = loco_obj->getConDes();
	memcpy(LLData0.ind_LL,contactMat,4*sizeof(int));

	//Debugging ptins
	// std::cout << "[LL] LLData0.q_LL: " << LLData0.q_LL[0] << ", " << LLData0.q_LL[1] << ", " << LLData0.q_LL[2] << ", " <<  LLData0.q_LL[3] << ", " << LLData0.q_LL[4] << ", " << LLData0.q_LL[5] << 
	// 			 ", " << LLData0.q_LL[6] << ", " << LLData0.q_LL[7] << ", " << LLData0.q_LL[8] << LLData0.q_LL[9] << ", " << LLData0.q_LL[10] << ", " << LLData0.q_LL[11] << std::endl;

	/* // State Estimation	// 
	for (int i=0; i<12; ++i){
		q[i+6] = LLData0.q[i+6];
		dq[i+6] = LLData0.dq[i+6];
	}
	discrete_butter_d(jointfilter,&dq[6]);

	// q[3] = LLData0; q[4] = state.imu.rpy[1]; q[5] = state.imu.rpy[2];
	quat_to_XYZ(LLData0.quaternion[0],LLData0.quaternion[1],LLData0.quaternion[2],LLData0.quaternion[3],
	            q[3],q[4],q[5]);
	dq[3] = LLData0.dq[3]; dq[4] = LLData0.dq[4]; dq[5] = LLData0.dq[5];

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(LLData0.quaternion,R);
	rotMat = R.data();

	// Kinematic Estimation
	int footForce[4];
	footForce[0] = LLData0.footForce[0]; footForce[1] = LLData0.footForce[1];
	footForce[2] = LLData0.footForce[2]; footForce[3] = LLData0.footForce[3];
	static int conIndDes[4] = {1,1,1,1};
	kinEst0(footForce,conIndDes,q,dq,R); // Defined in OtherUtils.hpp */
	
	// Quad Initialization Time //
	if ( (motiontime < SettlingTime) ){
		LLData0.jointTorqueFF.setZero();
	}
	// ================== LL Controller ==================== //
	static long int endStand = SettlingTime+StandDuration;
	
	if ( motiontime >= SettlingTime ){
		if (!softFall){
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = LLData0.q_LL[0], com(1) = LLData0.q_LL[1], com(2) = LLData0.q_LL[2];
				endStand = motiontime+StandDuration;
				loco_obj->initStandVars(com,StandDuration);
				setup = false;
			}
			
			beginPose = (motiontime<endStand) ? 0 : 1;
			if (beginPose == 0){
				
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
				loco_obj->calcZOHTauWithGRF(LLData0.q_LL,LLData0.dq_LL,LLData0.rotMatrixDouble,LLData0.footForce,STAND_MPC,motiontime, &(LLData0.runMPC));
				// loco_obj->calcTauLLonly(LLData0.q_LL,LLData0.dq_LL,LLData0.rotMatrixDouble,LLData0.footForce,STAND,motiontime, &(LLData0.runMPC));
				
				// loco_obj->calcZOHTauWithGRF(LLData0.q_LL,LLData0.dq_LL,LLData0.rotMatrixDouble,LLData0.footForce,STAND,motiontime, &(LLData0.runMPC));
			}
			else if (beginPose == 1 && motiontime>=endStand)
			{
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
				loco_obj->calcZOHTauWithGRF(LLData0.q_LL,LLData0.dq_LL,LLData0.rotMatrixDouble,LLData0.footForce,TROT,motiontime, &(LLData0.runMPC));
				// loco_obj->calcTauLLonly(LLData0.q_LL,LLData0.dq_LL,LLData0.rotMatrixDouble,LLData0.footForce,TROT,motiontime, &(LLData0.runMPC));
				//loco_obj->calcTauLLonly(q, dq, rotMat, footForce, TROT, motiontime,&(LLData0.runMPC));


				
				
				
			}
			LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
			LLData0.toePos = loco_obj->get_toePos();
            LLData0.domain = loco_obj->getDomain();
            LLData0.domain_change = loco_obj->hasDomainChanged();
			Eigen::Map<Eigen::VectorXd> tau = Eigen::Map<Eigen::VectorXd>(loco_obj->getTorque(),18);
			LLData0.jointTorqueFF = tau ;
			// std::cout << " [LL] jointTorqueFF: " << LLData0.jointTorqueFF << std::endl;
			

			if(LLData0.resetRun == -1){
				LLData0.resetRun = -2;
			}

			updateData(SET_DATA, LL_DATA, &LLData0, 0);

			/* // Set the command
			for (int i = 0; i < 12; ++i){
				LLData0.jointTorqueFF(i+6) = loco_obj->ll->tau[i + 6];
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
				LLData0.jointTorqueFF(i+6) = 0.0;
			}
		} */
	}
	}

	// const int* contactMat = loco_obj->getConDes();
	memcpy(LLData0.q_LL,q,18*sizeof(double));
    memcpy(LLData0.dq_LL,dq,18*sizeof(double));
	// memcpy(LLData0.ind_LL, contactMat,4*sizeof(int));

	// Set Updated data for MPC
	updateData(SET_DATA, LL_DATA, &LLData0, 0);

	// auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

	// std::cout << "Time taken by function: "<< duration.count() << " us" << std::endl;
}

void ExternalComm::SimExec(){

	// std::cout << "[SIM] RUNNING SIM (motiontime:) " << std::to_string(motiontime) << "\n";
	// auto start = std::chrono::high_resolution_clock::now();
	static sharedData SimData0;
	// Get the data for the simulation
	updateData(GET_DATA, SIM_DATA, &SimData0, 0);


	if (setup_raisim){

		raisim::World::setActivationKey(raisim::loadResource("~/.raisim/activation.raisim"));
		world.setTimeStep(simfreq_raisim);

		
		/// these method must be called before initApp
		vis->setWorld(&world);
		vis->setWindowSize(1792, 1200); // Should be evenly divisible by 16!!
		vis->setImguiSetupCallback(imguiSetupCallback); // These 2 lines make the interactable gui visible
		vis->setImguiRenderCallback(imguiRenderCallBack);
		vis->setKeyboardCallback(raisimKeyboardCallback);
		vis->setSetUpCallback(std::bind(&ExternalComm::setupCallback, this));
		vis->setAntiAliasing(2);

		/// starts visualizer thread
		vis->initApp();

		terrainProperties.frequency = 0.0;
		terrainProperties.zScale = 0.0;
		terrainProperties.xSize = 300.0;
		terrainProperties.ySize = 300.0;
		terrainProperties.xSamples = 50;
		terrainProperties.ySamples = 50;
		terrainProperties.fractalOctaves = 0;
		terrainProperties.fractalLacunarity = 0.0;
		terrainProperties.fractalGain = 0.0;

		ground = world.addHeightMap(0.0, 0.0, terrainProperties);

		vis->createGraphicalObject(ground, "terrain", "checkerboard_blue");
		world.setDefaultMaterial(0.8, 0.0, 0.0); //surface friction could be 0.8 or 1.0
		vis->addVisualObject("extForceArrow", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

		// Foot force visualization arrows
		vis->addVisualObject("GRF1", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
		vis->addVisualObject("GRF2", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
		vis->addVisualObject("GRF3", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
		vis->addVisualObject("GRF4", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
		list = &(vis->getVisualObjectList());

		A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new.urdf")));   // with NMPC and LL 
		// A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new_w_kinova.urdf")));
		// A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_trunk.urdf")));             // For NMPC only
		// A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova.urdf"))); 
		vis->createGraphicalObject(A1.back(), "A1");
		A1.back()->setName("A1_Robot");

		A1.back()->setGeneralizedCoordinate({0, 0, 0.12, 1 , 0, 0, 0,0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});

		A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

		// ============================================================ //
            // ================= VIEW AND RECORDING OPTIONS =============== //
            // ============================================================ //
		raisim::gui::showContacts = false;
		raisim::gui::showForces = false;
		raisim::gui::showCollision = false;
		raisim::gui::showBodies = true;
		// ============================================================ //
		// ========================= VIEW SETUP ======================= //
		// ============================================================ //
		if(cameraview == "iso"){
			vis->getCameraMan()->getCamera()->setPosition(-1, -3, 0.5);
			vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(4.5*Pi/6-Pi/2));
			vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
		}else if(cameraview == "isoside"){
			vis->getCameraMan()->getCamera()->setPosition(1.1, -2, 0.5);
			vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(4*Pi/6-Pi/2));
			vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
		}else if(cameraview == "side"){
			vis->getCameraMan()->getCamera()->setPosition(0, -2, 0.5);
			vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(0));
			vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
		}else if(cameraview == "front"){
			vis->getCameraMan()->getCamera()->setPosition(2, 0, 0.5);
			vis->getCameraMan()->getCamera()->yaw(Ogre::Radian(Pi/2));
			vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(Pi/2));
		}else if(cameraview == "top"){
			vis->getCameraMan()->getCamera()->setPosition(2, -1, 6);
			vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(0));
		}else{
			vis->getCameraMan()->getCamera()->setPosition(1, -3, 2.5);
			vis->getCameraMan()->getCamera()->pitch(Ogre::Radian(1.0));
		}
		unsigned long mask = 0;
		if(raisim::gui::showBodies) mask |= raisim::OgreVis::RAISIM_OBJECT_GROUP;
		if(raisim::gui::showCollision) mask |= raisim::OgreVis::RAISIM_COLLISION_BODY_GROUP;
		if(raisim::gui::showContacts) mask |= raisim::OgreVis::RAISIM_CONTACT_POINT_GROUP;
		if(raisim::gui::showForces) mask |= raisim::OgreVis::RAISIM_CONTACT_FORCE_GROUP;
		vis->setVisibilityMask(mask);

		if(panX) raisim::gui::panViewX = panX;
		if(panY) raisim::gui::panViewY = panY;

		const std::string name = directory+filename+"_"+cameraview+".mp4";
		vis->setDesiredFPS(fps);

		setup_raisim = false;


	}
	if (!vis->getRoot()->endRenderingQueued() && simcounter < simlength-1){
		
		
		
		//Set the joint torques
		jointTorqueFF = SimData0.jointTorqueFF;
		jointTorqueFF.block(0,0,6,1).setZero(); //Raisim has torque with each DOF, so we need to set the first 6 (SRB COM) to zero
		// std::cout << "[SIM] jointTorqueFF: " << jointTorqueFF << std::endl;
		// Set the desired torques
		A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

		A1.back()->setGeneralizedForce(jointTorqueFF);

		std::vector<double> GRF(std::begin(SimData0.HL_GRF), std::end(SimData0.HL_GRF));
		std::vector<double> feet_vec(std::begin(SimData0.HL_feet_vec), std::end(SimData0.HL_feet_vec));
		std::vector<double> contacts(std::begin(SimData0.HL_contacts), std::end(SimData0.HL_contacts));
		plotGRFs(list, GRF, feet_vec, contacts);	

		world.integrate();        
		
		if (simcounter%30 == 0)
			vis->renderOneFrame();
		
		
		if (!vis->isRecording() & record & simcounter>=startTime)
			vis->startRecordingVideo(name);
		
		auto currentPos = vis->getCameraMan()->getCamera()->getPosition();
		if (raisim::gui::panViewX){
			Eigen::VectorXd jointPosTotal(18 + 1);
			Eigen::VectorXd jointVelTotal(18);
			jointPosTotal.setZero();
			jointVelTotal.setZero();
			A1.back()->getState(jointPosTotal, jointVelTotal);
			if (cameraview=="front"){
				currentPos[0] = jointPosTotal(0)+2;
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			} else if(cameraview=="side"){
				currentPos[0] = jointPosTotal(0);
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			} else if(cameraview=="iso"){
				currentPos[0] = jointPosTotal(0)+2;
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			} else if(cameraview=="isoside"){
				currentPos[0] = jointPosTotal(0)+1.1;
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			}
		}
		if (raisim::gui::panViewY){
			Eigen::VectorXd jointPosTotal(18 + 1);
			Eigen::VectorXd jointVelTotal(18);
			jointPosTotal.setZero();
			jointVelTotal.setZero();
			A1.back()->getState(jointPosTotal, jointVelTotal);
			if (cameraview=="front"){
				currentPos[1] = jointPosTotal(1);
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			} else if(cameraview=="side"){
				currentPos[1] = jointPosTotal(1)-2;
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			} else if(cameraview=="iso"){
				currentPos[1] = jointPosTotal(1)-1;
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			} else if(cameraview=="isoside"){
				currentPos[1] = jointPosTotal(1)-2;
				vis->getCameraMan()->getCamera()->setPosition(currentPos);
			}
		}
		simcounter++;
	}
	else if (simcounter >= simlength-1){
		vis->closeApp();
	}

	A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat;
    Eigen::Matrix3d rotE;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }
    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    double jpos[18], jvel[18];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul); // Convert quaternion to Euler angles
    Eigen::VectorXd noise_pos = 0*0.01*Eigen::VectorXd::Random(18);
    Eigen::VectorXd noise_vel = 0*0.15*Eigen::VectorXd::Random(18);
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i) + noise_pos(i);
        jvel[i] = jointVelTotal(i) + noise_pos(i);
        jpos[i+3] = eul(i) + noise_pos(i+3);
        jvel[i+3] = jointVelTotal(i+3) + noise_pos(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1) + noise_pos(i);
        jvel[i] = jointVelTotal(i) + noise_pos(i);
    }
//Setting Retrieving contact positions and forces from Randy. con is an object of a Randy's struct - it has all the info about footholds
    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 0;
        // force[conInd/3-1] = con.getNormal().e().norm();
    }
// Update data in the shared struct for LL controller
    memcpy(SimData0.q_LL,jpos,18*sizeof(double));
    memcpy(SimData0.dq_LL,jvel,18*sizeof(double));
	memcpy(SimData0.rotMatrixDouble,rotMatrixDouble,9*sizeof(double));
	memcpy(SimData0.footForce,force,4*sizeof(int));

    // const int* contactMat = loco_obj->getConDes();
    // memcpy(SimData0.ind_LL,contactMat,4*sizeof(int));
    // memcpy(conIndDes,contactMat,4*sizeof(int));

	// Set Updated data for MPC/LL
	updateData(SET_DATA, SIM_DATA, &SimData0, 0);  

	//Debugging ptins
	// std::cout << "[SIM] SimData0.q_LL: " << SimData0.q_LL[0] << ", " << SimData0.q_LL[1] << ", " << SimData0.q_LL[2] << ", " <<  SimData0.q_LL[3] << ", " << SimData0.q_LL[4] << ", " << SimData0.q_LL[5] << 
	// 			 "," << SimData0.q_LL[6] << ", " << SimData0.q_LL[7] << ", " << SimData0.q_LL[8] << SimData0.q_LL[9] << ", " << SimData0.q_LL[10] << ", " << SimData0.q_LL[11] << std::endl;

	// auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

	// std::cout << "Time taken by [SIM]: "<< duration.count() << " us" << std::endl;

	

}


int main(int argc, char *argv[])
{
	// InitEnvironment();
	/* struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = 99;
    if(sched_setscheduler(0, SCHED_FIFO, &sp)==-1)
    std::cerr << "Error: Pthread set sched policy failed: " << std::strerror(errno) << std::endl; */

	ExternalComm extComm;

	Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart;

	// Pstart << 0.0, 0.0, 0, -1.5;//, 1.0, -2, 1, -3;	//2.0, -2.0
	
    Pstart << 0.0, 0.0; //, 1.0, -2, 1, -3;	//2.0, -2.0
	

	extComm.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv,true)); //isSim = False
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
	// LoopFunc loop_mpc("mpc_loop", 0.010001f,2, boost::bind(&ExternalComm::HighLevel, &extComm));
	// LoopFunc loop_sim("sim_loop", extComm.dt,3, boost::bind(&ExternalComm::SimExec, &extComm));
	
	// loop_sim.start();
	// sleep(1.0);
	// loop_mpc.start();
	// sleep(1.0);
	// loop_calc.start();
	// sleep(0.01);
	
	// auto start = std::chrono::high_resolution_clock::now();
	bool sim_setup = true;
	while (true)
	{
		if (sim_setup)
		{		
			extComm.SimExec();
			// sleep(0.1); //cin>>get to pause the program
			extComm.HighLevel();;
			// sleep(0.1);
			extComm.Calc();
			// sim_setup = false;
		}
	}
	// 	else
	// 	{
	// 	 	extComm.SimExec();
	// 	 	extComm.Calc();
	//  }
		
	// while(true)
	// {
	// 	sleep(0.001);
	// }
	// 	// auto end = std::chrono::high_resolution_clock::now();
    // 	// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	// 	// std::cout << "[LL] real time: "
	// 	// 	 << duration.count() << " milliseconds" << std::endl;
	return 0;
}
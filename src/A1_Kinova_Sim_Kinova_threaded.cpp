//
// Authror: Basit M. Imran.
// Date : 2024-04-17
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "raisim/OgreVis.hpp"
// #include "randyImguiPanel.hpp"
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include "helper2.hpp"
#include "MPC_dist.hpp"
#include "a1_kinova_nmpc.hpp"
#include "OtherUtils.hpp"
#include "timer.h"
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
//HDSRL header
#include "LocoWrapper.hpp"
#include "shared_structs.hpp"
#include "JoyStick.hpp"

#include "Transforms.hpp"
#include "Filters.h"
#include "OtherUtils.hpp"
#include <yaml-cpp/yaml.h>
#include "stdio.h"

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "gamepad.hpp"
#include <unitree/idl/ros2/String_.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

using namespace UNITREE_LEGGED_SDK;
//Kinova Header
#include <KinovaArm.hpp>
bool rough_terrain_en = 0;
bool obstacle_en = 0;

using std::cout;
using std::cin;


bool sec_time = 1;
sharedData HLData1;
sharedData LLData1;
sharedData SimData1;
sharedData KinovaData1;
sharedData JoyStickData1;

class ExternalComm {
public:
		ExternalComm(){
		
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

            //4 Hz 
            double av[5] = { 1.000000000, -3.934325821, 5.805125421,-3.807232457, 0.936433243}; 
			double bv[5] = {0.000000024, 0.000000097, 0.000000145,0.000000097, 0.000000024}; 
            populate_filter_d(linearvelfilter, av, bv, 5, 3);

			StandDuration = 4000;
			SettlingTime = 8000;
            SettlingTimeKinova = SettlingTime;
            settling = 8*ctrlHz;                   // Settling down
            duration = 4*ctrlHz;                   // Stand up
            stand_settle = 1*ctrlHz; // Settle down after stand up
            settling_nmpc = settling;                   // Settling down
            duration_nmpc = duration;                   // Stand up
            trot_start = settling + duration + stand_settle; //This is where we can play starting trot
			/********************************************RASIM INIT********************************************** */
			const int NUMBER_OF_SIMS = 1;
			const float threshold = 0.4;
			bool shared_data_backed_up = 0;
			vis = raisim::OgreVis::get();
            tau_arm =  {0, 9.94638, -3.12554, 0};//intialize arm torques to zero			
		}
	
		

	virtual ~ExternalComm(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);
        clear_filter_d(linearvelfilter);

		nmpc_obj->logData();
        nmpc_obj.reset();
        loco_obj.reset();
        kinova.reset();
        joystick.reset();
        delete vis;
        delete ground;
        delete list;
        auto test = A1.back();
        A1.pop_back();
        delete test;
	}


	//support functions
	void setupCallback();
	void plotGRFs(std::map<std::string, raisim::VisualObject>* list, const std::vector<double>& GRF, const std::vector<double>& feet_vec, const std::vector<double>& contacts);
	std::string getCurrentTimestamp();
    void setupRaisim();

	// main thread execution functions
	void Calc();
	void HighLevel();
	void SimExec();             
    void controller0();
    void runKinova();
    void holdKinova();
	void exitKinova();
    void runPDFFKinova();
    void ArmPDControl(std::vector<double> &tau_arm, double arm_pos[4], double arm_vel[4]);
    void JoystickLoop();

	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<NMPC_A1_KINOVA> nmpc_obj;
    std::unique_ptr<MovingAverageFilter> moving_avg_filter;
    std::unique_ptr<KinovaArm> kinova;
	std::unique_ptr<JoyStick> joystick;

    ThreadPtr CalcThread;
    ThreadPtr NMPCThread;
    ThreadPtr SimExecThread;
	ThreadPtr KinovaThread;
    ThreadPtr JoystickThread;

    
	long motiontime = 0;
    long simcounter = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;

    bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};

    

	int StandDuration;
	int SettlingTime;
    int SettlingTimeKinova;
    

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
    FiltStruct_d* linearvelfilter = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
    

	int stop = 0;
	float vel[3] = {0};
    double filtered_vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

    std::vector<double> GRF;
    std::vector<double> feet_vec;
    std::vector<double> contacts;

	float dt = 0.001f;
 // Some parameters in controller0
	size_t loco_kind = STAND_MPC;                        // Gait pattern to use
    size_t settling;                  // Settling down
    size_t duration;                   // Stand up
    size_t settling_nmpc;                   // Settling dow
    size_t loco_start = 0;        // Start the locomotion pattern
    size_t duration_nmpc;                   // Stand up
    size_t trot_start;
    size_t stand_settle = 20*ctrlHz; // Settle down after stand up 
    timer tset;

    double *tau;
    std::vector<double> tau_arm;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF_A1_KINOVA+6,1);
    Eigen::VectorXd jointTorqueA1 = Eigen::MatrixXd::Zero(TOTAL_DOF_A1,1);
    Eigen::Vector4d jointTorqueKinova = Eigen::MatrixXd::Zero(TOTAL_DOF_KINOVA,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF_A1_KINOVA+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF_A1_KINOVA,1);
    double jpos[18],jpos_est[18], jvel[18],jvel_est[18];
    double jpos_arm[4], jvel_arm[4];
    int conIndDes[4] = {1,1,1,1};
    Eigen::Matrix<double,6,1> gripperTorque = 0.1*Eigen::Matrix<double,6,1>::Ones(6,1);
	//Raisim stuff
	bool setup_raisim = true;
	raisim::World world;
	raisim::OgreVis *vis;
	raisim::TerrainProperties terrainProperties;
	raisim::HeightMap *ground;
	std::map<std::string, raisim::VisualObject>* list = nullptr;
	std::vector<raisim::ArticulatedSystem*> A1;
    bool checkifrunMPC0=1;
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
	std::string filename = "A1_KINOVA_NMPC_"+timestamp;
	std::string name = directory+filename+"_"+cameraview+".mp4";
    bool first_time_torque = true;

    float joystick_state[18] = {0};



};

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



void writeMatrixToFile(const Eigen::MatrixXd &matrix, const std::string &filename)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        // Print matrix dimensions
        // file << "Matrix size: " << matrix.size1() << "x" << matrix.size2() << std::endl;
        // Print matrix content preserving original shape
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                file << matrix(i, j);
                if (j < matrix.cols() - 1)
                    file << ", ";
            }
            file << std::endl;
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}



void ExternalComm::setupCallback() {

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

void disturbance(std::vector<raisim::ArticulatedSystem *> A1,std::map<std::string, raisim::VisualObject> &objList,size_t start,size_t stop, size_t ctrlTick){
    Eigen::VectorXd pos = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd vel = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    A1.back()->getState(pos, vel);
    
    // ====================== External Forces ====================== //
    raisim::Vec<3> extForce;
    raisim::Mat<3,3> rot;
    raisim::Vec<3> dir;
    if (ctrlTick>=start && ctrlTick<stop){
        extForce = {0,-20,0}; // Pulse
        // extForce = {50*sin(4*ctrlTick*simfreq_raisim),0,0}; // Fwd Sine
        //extForce = {0,20*sin(4*ctrlTick*simfreq_raisim),0}; // Lat Sine
    } else{
        extForce = {0,0,0};
    }
    A1.back()->setExternalForce(1,extForce);
    dir = extForce;
    dir /= dir.norm();
    raisim::zaxisToRotMat(dir, rot);
    objList["extForceArrow"].offset = {pos(0),pos(1),pos(2)};
    objList["extForceArrow"].scale = {0.2,0.2,0.01*extForce.norm()};
    objList["extForceArrow"].rotationOffset = rot;
}

void ExternalComm::plotGRFs(std::map<std::string, raisim::VisualObject>* list, const std::vector<double>& GRF, const std::vector<double>& feet_vec, const std::vector<double>& contacts) {
    // Ensure the vectors are of the correct size
    if (GRF.size() < 12 || feet_vec.size() < 12 || contacts.size() < 4) {
        std::cerr << "Error: Input vectors are of incorrect size." << std::endl;
        return;
    }

    for (int i = 0; i < 4; ++i) {
        raisim::Vec<3> dir;
        for (int j = 0; j < 3; ++j) {
            dir[j] = GRF[3 * i + j];
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
        if (contacts[i] == 1 && norm > 1e-6) {
            raisim::zaxisToRotMat(dir, rot);
        } else {
            rot.setIdentity();  // Set rotation to identity if no contact or zero norm

        }

        // Visual object key
        std::string objKey = "GRF" + std::to_string(i + 1);
        (*list)[objKey].offset = {feet_vec[3 * i], feet_vec[3 * i + 1], feet_vec[3 * i + 2]};
        (*list)[objKey].scale = {0.2, 0.2, 0.005 * norm};  // Scaling based on the norm of the GRF vector

        // Set the rotation matrix
        (*list)[objKey].rotationOffset = rot;

    }
}


void ExternalComm::ArmPDControl(std::vector<double> &tau_arm, double arm_pos[4], double arm_vel[4]){
    // PD control for the Kinova arm
    double Kp[4] = {3252,500,50,2}; // Proportional gain
    double Kd[4] = {30,40,15,0.2}; // Derivative gain
    /* double Kp[4] = {325.2,50.0,50,2}; // Proportional gain
    double Kd[4] = {10,4.0,15,0.2}; // Derivative gain
 */
    // Desired position and velocity
    double desired_pos[4] = {0.0,3.49066,5.67232,0.0};// {0.0,2.68, 1.051,0.0};//{0.0,1.13,2.45,0.0}; // Set your desired position here

    // PD control law
    for (int i = 0; i < tau_arm.size(); ++i) {
        tau_arm[i] = Kp[i] * (desired_pos[i] - arm_pos[i]) + Kd[i] * (0 - arm_vel[i]);
    }
}

void ExternalComm::JoystickLoop() {
    // Joystick control loop]
    joystick->getJoyShockState(JoyStickData1.joystick_state);
    updateData(SET_DATA, JOYSTICK_DATA, &JoyStickData1 , 1);  
       
}

void ExternalComm::HighLevel(){
    auto start = std::chrono::high_resolution_clock::now();

    updateData(GET_DATA, HL_DATA, &HLData1,1);
    // if (HLData1.runMPC)
    if (true)
        {
            // std::cout << "[Running MPC]" << std::endl;

           /*  filtered_vel[0] = HLData1.dq[0];
            filtered_vel[1] = HLData1.dq[1];
            filtered_vel[2] = HLData1.dq[2];
 */         
            // std::cout << HLData1.q_arm[0] << ',' << HLData1.q_arm[1] << ',' << HLData1.q_arm[2] << ',' << HLData1.q_arm[3] << std::endl;
            // discrete_butter_d(linearvelfilter, &filtered_vel[0]);
            // std::cout << filtered_vel[0] << "\t" << filtered_vel[1] << "\t" << filtered_vel[2] << std::endl;           
            nmpc_obj->updateState(HLData1.q, HLData1.dq, HLData1.q_arm, HLData1.dq_arm, HLData1.ind, HLData1.toePos, HLData1.last_state1, HLData1.domain, HLData1.control_tick);
            nmpc_obj->updateReference(HLData1.joystick_state);
            HLData1.runMPC = 0;
            
            // std::cout << "HLData1.index: HLData.control_tick "<< HLData1.control_tick << "\t" << HLData1.ind_LL[0] << "\t" << HLData1.ind_LL[1] << "\t" << HLData1.ind_LL[2] << "\t" << HLData1.ind_LL[3] << std::endl;
                                         
            HLData1.MPC_data_available = 1;
            nmpc_obj->run_NMPC();
            
            HLData1.MPC_data_available = 1;
            HLData1.alpha_COM = nmpc_obj->get_alphaCOM();
            HLData1.alpha_GRF = nmpc_obj->get_alphaGRF();
            HLData1.MPC_sol_ = nmpc_obj->get_MPCsol();
            HLData1.MPC_sol_GRF_ = nmpc_obj->get_MPCsolGRF();
            tau_arm = nmpc_obj->get_arm_input_torques();
            if (HLData1.control_tick <= settling_nmpc)
            {
                
                // tau_arm = {0, 9.94638, -3.12554, 0};
                // tau_arm = {0, 0,0, 0};
                ArmPDControl(tau_arm, HLData1.q_arm, HLData1.dq_arm);


            }
            HLData1.jointTorqueKinova = Eigen::Map<Eigen::Matrix<double,4,1>>(tau_arm.data(), tau_arm.size());
            
            std::vector<double> temp_contacts = nmpc_obj->get_contacts();
		    std::vector<double> temp_Pf = nmpc_obj->get_Pf();
            std::vector<double> temp_GRF = nmpc_obj->get_GRF();  

            std::copy(temp_contacts.begin(), temp_contacts.end(), HLData1.HL_contacts);
            std::copy(temp_Pf.begin(), temp_Pf.end(), HLData1.HL_feet_vec);
            std::copy(temp_GRF.begin(), temp_GRF.end(), HLData1.HL_GRF);

            updateData(SET_DATA, HL_DATA, &HLData1,1);
        }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "HighLevel Time: " << duration.count() << " us" << std::endl;
}

void ExternalComm::Calc(){

    updateData(GET_DATA, LL_DATA, &LLData1,1);

    // std::cout<<"[Running LL]"<<std::endl;
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(LLData1.control_tick < settling){ // Settle down
        double temp[18] = {0};
        Eigen::Map<Eigen::VectorXd> tau1 = Eigen::Map<Eigen::VectorXd>(temp,18);
        LLData1.jointTorqueFF = tau1 ;
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration);
        memcpy(LLData1.q,LLData1.q_LL,18*sizeof(double));
        memcpy(LLData1.dq,LLData1.dq_LL,18*sizeof(double));
        updateData(SET_DATA, LL_DATA, &LLData1,1);
    }
    else
    {   
        // LLData1.runMPC = 1;
        if(LLData1.control_tick <= loco_start) 
        {
            loco_kind = STAND_MPC;
        }
        else if(LLData1.control_tick <= trot_start)
        {
            loco_kind = STAND_MPC;
        }
        else
        {
            if (LLData1.gait==0)
            {
                loco_kind = STAND_MPC;
            }
            else
            {
                loco_kind = TROT;
            }

        }



        loco_obj->set_MPC_DATA_W_GRF(LLData1.alpha_COM, LLData1.MPC_sol_, LLData1.alpha_GRF, LLData1.MPC_sol_GRF_,LLData1.MPC_data_available);
        loco_obj->calcZOHTauWithGRF(LLData1.q_LL,LLData1.dq_LL,LLData1.rotMatrixDouble,LLData1.footForce,loco_kind,LLData1.control_tick, &(LLData1.runMPC));
        // loco_obj->calcTauLLonly(LLData1.q_LL,LLData1.dq_LL,LLData1.rotMatrixDouble,LLData1.footForce,loco_kind,LLData1.control_tick, &(LLData1.runMPC));
        Eigen::Map<Eigen::VectorXd> tau2 = Eigen::Map<Eigen::VectorXd>(loco_obj->getTorque(),18);
        LLData1.jointTorqueA1 = tau2;
        LLData1.jointTorqueA1.block(0,0,6,1).setZero(); //Raisim has torque with each DOF, so we need to set the first 6 (SRB COM) to zero
        LLData1.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        LLData1.toePos = loco_obj->get_toePos();
        LLData1.domain = loco_obj->getDomain();
        LLData1.domain_change = loco_obj->hasDomainChanged();
        memcpy(LLData1.ind, contactMat,4*sizeof(int));
        memcpy(LLData1.ind_LL, contactMat,4*sizeof(int));
        memcpy(LLData1.q,LLData1.q_LL,18*sizeof(double));
        memcpy(LLData1.dq,LLData1.dq_LL,18*sizeof(double));
        //Filtering the linear velocity
        // moving_avg_filter->filter(&LLData1.dq[0]);
        //Filtering the position
        // moving_avg_filter->filter(&LLData1.q[0]);
        // discrete_butter_d(linearvelfilter,&LLData1.dq[0]);
        updateData(SET_DATA, LL_DATA, &LLData1,1);
    }
}

void ExternalComm::setupRaisim()
{
    raisim::World::setActivationKey(raisim::loadResource("activation.raisim"));
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
    /// create raisim objects
    raisim::TerrainProperties terrainProperties;
    terrainProperties.frequency = 0.0;
    terrainProperties.zScale = 0.0;
    terrainProperties.xSize = 300.0;
    terrainProperties.ySize = 300.0;
    terrainProperties.xSamples = 50;
    terrainProperties.ySamples = 50;
    terrainProperties.fractalOctaves = 0;
    terrainProperties.fractalLacunarity = 0.0;
    terrainProperties.fractalGain = 0.0;

    raisim::HeightMap *ground = world.addHeightMap(0.0, 0.0, terrainProperties);
    vis->createGraphicalObject(ground, "terrain", "checkerboard_blue");
    world.setDefaultMaterial(0.8, 0.0, 0.0); //surface friction could be 0.8 or 1.0
    vis->addVisualObject("extForceArrow", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

    // Foot force visualization arrows
    vis->addVisualObject("GRF1", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
    vis->addVisualObject("GRF2", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
    vis->addVisualObject("GRF3", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);
    vis->addVisualObject("GRF4", "arrowMesh", "red", {0.0, 0.0, 0.0}, false, raisim::OgreVis::RAISIM_OBJECT_GROUP);

    list = &(vis->getVisualObjectList());
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new_w_kinova.urdf")));   // with NMPC and LL
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova.urdf")));   // with NMPC and LL  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_collision.urdf")));   // with NMPC and LL  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_fingers.urdf")));   // with NMPC and LL  
    A1.push_back(world.addArticulatedSystem(raisim::loadResource("Go2/go2_kinova_modified1.urdf")));   // with NMPC and LL  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_fingers.urdf")));
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_light.urdf")));   // with NMPC and LL with light URDF  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_trunk.urdf")));             // For NMPC only
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new.urdf"))); 
    
    vis->createGraphicalObject(A1.back(), "A1");
    A1.back()->setName("A1_Robot");
    A1.back()->setGeneralizedCoordinate({0.0, 0.0, 0.12, 1, 0, 0, 0,0.0, Pi/3, -2.8, 0.0, Pi/3, -2.8, 0.0, Pi/3, -2.8, 0.0, Pi/3, -2.8, 0.0,3.49066,5.67232,0.0, 0.75,0.75,0.75,0.75,0.75,0.75 });
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
   
    raisim::gui::showContacts = false;
    raisim::gui::showForces = false;
    raisim::gui::showCollision = false;
    raisim::gui::showBodies = true;

    static std::string cameraview = "iso"; // iso, isoside, side, front, top
    bool panX = true;                // Pan view with robot during walking (X direction)
    bool panY = true;                // Pan view with robot during walking (Y direction)
    static bool record = false;             // Record?
    double startTime = 0*ctrlHz;    // Recording start time
    double simlength = SIM_TIME*ctrlHz;
    double fps = 30;            
    std::string directory = "~/OneDrive/1Research_legged_locomotion/A1_Kinova_locomanipulation/raisim/";
    std::string timestamp = getCurrentTimestamp();
    std::string filename = "A1_KINOVA_NMPC_"+timestamp;

    if(panX) raisim::gui::panViewX = panX;
		if(panY) raisim::gui::panViewY = panY;

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
    
    // ============================================================ //
    // ========================== RUN SIM ========================= //
    // ============================================================ //
    const std::string name = directory+filename+"_"+cameraview+".mp4";
    vis->setDesiredFPS(fps);
    static bool added = false;

    loco_start = settling + duration;   

}

void ExternalComm::SimExec(){
 
    if (setup_raisim){
        setupRaisim();
        setup_raisim = false;
    }
    updateData(GET_DATA, SIM_DATA, &SimData1,1);
    if (!vis->getRoot()->endRenderingQueued() && simcounter < simlength-1){
		
		// std::cout << "[Running Simulation]" << std::endl;
		//Set the joint torques
		
        jointTorqueFF.block(0,0,18,1) = SimData1.jointTorqueA1;
        jointTorqueFF.block(0,0,6,1).setZero(); //Raisim has torque with each DOF, so we need to set the first 6 (SRB COM) to zero
        // jointTorqueFF.block(0,0,18,1) = jointTorqueA1;
        // jointTorqueFF.block(0,0,18,1).setZero();
        jointTorqueFF.block(18,0,4,1) = SimData1.jointTorqueKinova;
        //Gripper Torque
        Eigen::Matrix<double,6,1> gripperTorque = 0.1*Eigen::Matrix<double,6,1>::Ones();
        //Combine the torques for a1 kinmona and gripper
        jointTorqueFF.block(22,0,6,1) = gripperTorque; 
            
		
		// std::cout << "[SIM] jointTorqueFF: " << jointTorqueFF << std::endl;
		// Set the desired torques
		A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

		A1.back()->setGeneralizedForce(jointTorqueFF);

		std::vector<double> GRF(std::begin(SimData1.HL_GRF), std::end(SimData1.HL_GRF));
		std::vector<double> feet_vec(std::begin(SimData1.HL_feet_vec), std::end(SimData1.HL_feet_vec));
		std::vector<double> contacts(std::begin(SimData1.ind_LL), std::end(SimData1.ind_LL));
		if(GRF.size() >0) plotGRFs(list, GRF, feet_vec, contacts);
	

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
    //////////////////////////////////
    //      STATE ESTIMATION        //
    //////////////////////////////////

	A1.back()->getState(jointPosTotal, jointVelTotal);
    raisim::Mat<3,3> rotMat,rotMat2;
    Eigen::Matrix3d rotE;
    A1.back()->getBaseOrientation(rotMat);
    double rotMatrixDouble[9];
    for(size_t i=0;i<9;i++){
        rotMatrixDouble[i] = rotMat[i];
        rotE(i) = rotMat[i];
    }


    jointVelTotal.segment(3,3) = rotE.transpose()*jointVelTotal.segment(3,3); // convert to body frame, like robot measurements

    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul); // Convert quaternion to Euler angles
    
    
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);

        jpos[i+3] = eul(i);
        jpos_est[i+3] = eul(i);

        jvel[i+3] = jointVelTotal(i+3);
        jvel_est[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jpos_est[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
        jvel_est[i] = jointVelTotal(i);
    }

    for (size_t i = 0; i < 4; ++i) {
        jpos_arm[i] = jointPosTotal(i + 18 + 1);
        jvel_arm[i] = jointVelTotal(i + 18);
    }
    // discrete_butter_d(jointfilter,&jvel_est[6]);
    
    double *rotMatrixDouble2;
    Eigen::Matrix3d rotE2;
    quat_to_R(quat,rotE2);

    //Setting Retrieving contact positions and forces from Randy. con is an object of a Randy's struct - it has all the info about footholds
    int force[4]{0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        // force[conInd/3-1] = con.getNormal().e().norm();
        force[conInd/3-1] = 0;
    }

    int footForce[4];
    footForce[0] = force[0];
    footForce[1] = force[1];
    footForce[2] = force[2];
    footForce[3] = force[3];

    kinEst0(force,SimData1.ind_LL,jpos_est,jvel_est,rotE2);
    rotMatrixDouble2 = rotE2.data();
    // std::cout << "\rdelta_jpos: " << ([&](){ std::ostringstream oss; for (int i = 0; i < 18; ++i) oss << jpos_est[i]-jpos[i] << " "; return oss.str(); }()) << std::flush;
   /*  std::cout << "\rdelta_jpos: " << ([&](){
        std::ostringstream oss;
        for (int i = 0; i < 18; ++i)
            oss << std::setw(8) << std::fixed << std::setprecision(6) << jpos_est[i]-jpos[i] << " ";
        return oss.str();
    }()) << std::flush; */
    // Add noise to jvel_est
    Eigen::VectorXd noise_vel = 0*0.15*Eigen::VectorXd::Random(18);
    for (size_t i = 0; i < 18; i++)
    {
        jvel_est[i] += noise_vel[i];
    }
    
    // discrete_butter_d(linearvelfilter,&jvel_est[0]);
// Update data in the shared struct for LL controller
    // memcpy(SimData1.q_LL,jpos,18*sizeof(double));
    // memcpy(SimData1.dq_LL,jvel,18*sizeof(double));
	// memcpy(SimData1.rotMatrixDouble,rotMatrixDouble,9*sizeof(double));
    memcpy(SimData1.q_LL,jpos_est,18*sizeof(double));
    memcpy(SimData1.dq_LL,jvel_est,18*sizeof(double));
    memcpy(SimData1.q_arm,jpos_arm,4*sizeof(double));
    memcpy(SimData1.dq_arm,jvel_arm,4*sizeof(double));
	memcpy(SimData1.rotMatrixDouble,rotMatrixDouble2,9*sizeof(double));
	memcpy(SimData1.footForce,force,4*sizeof(int));
    SimData1.control_tick = simcounter;
    // const int* contactMat = loco_obj->getConDes();
    // memcpy(SimData1.ind_LL,contactMat,4*sizeof(int));
    // memcpy(conIndDes,contactMat,4*sizeof(int));

	// Set Updated data for MPC/LL
	updateData(SET_DATA, SIM_DATA, &SimData1, 1);  


}

void ExternalComm::runKinova(){
    auto start = std::chrono::high_resolution_clock::now();
    //Get Kinova Data
    Eigen::Matrix<double, 4, 1> tau_arm = Eigen::Matrix<double, 4, 1>::Zero();
    
    updateData(GET_DATA, KINOVA_DATA, &KinovaData1,1);
    if (KinovaData1.control_tick >= SettlingTimeKinova){        
        
        // kinova->runTorqueControl(KinovaData1.jointTorqueKinova);
        // tau_arm(2) = (KinovaData1.control_tick>= SettlingTimeKinova + 5000) ? std::max(-(4*0.25/100)*((KinovaData1.control_tick-SettlingTimeKinova-5000)/10.0),-5.0) : 0;
        // tau_arm(1) = (KinovaData1.control_tick>= SettlingTimeKinova + 5000) ? std::min((4*0.25/100)*((KinovaData1.control_tick-SettlingTimeKinova-5000)/10.0),16.0) : 0;
        
        tau_arm(1) = 5;// 8.5;
        tau_arm(2) = -3.0;
        tau_arm(3) = 2.0;
        std::cout << "tau_arm joint 2: " << tau_arm(1) << std::endl;
        std::cout << "tau_arm joint 3: " << tau_arm(2) << std::endl;
        // tau_arm(3) = 0;
        kinova->runTorqueControl(tau_arm);
        kinova->sendGripperCommand(KinovaData1.joystick_state);
    }
    

    kinova->getAngularPosition(KinovaData1.q_arm);
    kinova->getAngularVelocity(KinovaData1.dq_arm);
    
    // std::cout << KinovaData1.q_arm[0] << ',' << KinovaData1.q_arm[1] << ',' << KinovaData1.q_arm[2] << ',' << KinovaData1.q_arm[3] << std::endl;
    std::cout << "Kinova Control Tick: " << KinovaData1.control_tick << std::endl;
    updateData(SET_DATA, KINOVA_DATA, &KinovaData1,0);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Kinova Time: " << duration.count() << " us" << std::endl;
}


void ExternalComm::runPDFFKinova(){
    auto start = std::chrono::high_resolution_clock::now();
    //Get Kinova Data
    Eigen::VectorXd tau_arm = Eigen::VectorXd::Zero(4);
    
    updateData(GET_DATA, KINOVA_DATA, &KinovaData1,1);
    if (KinovaData1.control_tick >= SettlingTimeKinova){        
        
        // kinova->runTorqueControl(KinovaData1.jointTorqueKinova);
        // tau_arm(2) = (KinovaData1.control_tick>= SettlingTimeKinova + 5000) ? std::max(-(4*0.25/100)*((KinovaData1.control_tick-SettlingTimeKinova-5000)/10.0),-5.0) : 0;
        // tau_arm(1) = (KinovaData1.control_tick>= SettlingTimeKinova + 5000) ? std::min((4*0.25/100)*((KinovaData1.control_tick-SettlingTimeKinova-5000)/10.0),16.0) : 0;
        kinova->getPDTorques(tau_arm);
  
        //Feedforward torque
        tau_arm(0)+= 0;
        tau_arm(1)+= 10;
        tau_arm(2)+= -4;
        tau_arm(3)+= 0;

        std::cout <<"tau_arm joint 1: " << tau_arm(0)<< ", tau_arm joint 2: " << tau_arm(1) << ", tau_arm joint 3: " << tau_arm(2) << ", tau_arm joint 4: " << tau_arm(3) << std::endl;
        
        tau_arm(0) = 0;
        tau_arm(1) = 0;
        tau_arm(2) = 0;
        tau_arm(3) = 0;
        
        kinova->runTorqueControl(tau_arm);
    }
    

    kinova->getAngularPosition(KinovaData1.q_arm);
    kinova->getAngularVelocity(KinovaData1.dq_arm);
    
    // std::cout << KinovaData1.q_arm[0] << ',' << KinovaData1.q_arm[1] << ',' << KinovaData1.q_arm[2] << ',' << KinovaData1.q_arm[3] << std::endl;
    std::cout << "Kinova Control Tick: " << KinovaData1.control_tick << std::endl;
    updateData(SET_DATA, KINOVA_DATA, &KinovaData1,0);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Kinova Time: " << duration.count() << " us" << std::endl;
}

void ExternalComm::holdKinova(){
    //Get Kinova Data
    updateData(GET_DATA, KINOVA_DATA, &KinovaData1,0);
    
    kinova->holdPosition();

    kinova->getAngularPosition(KinovaData1.q_arm);
    kinova->getAngularVelocity(KinovaData1.dq_arm);
    
    std::cout << KinovaData1.q_arm[0] << ',' << KinovaData1.q_arm[1] << ',' << KinovaData1.q_arm[2] << ',' << KinovaData1.q_arm[3] << std::endl;
    // std::cout << "Kinova Velocity: " << KinovaData1.dq_arm[0] << ", " 
    //           << KinovaData1.dq_arm[1] << ", " 
    //           << KinovaData1.dq_arm[2] << ", " 
    //           << KinovaData1.dq_arm[3] << std::endl;

    updateData(SET_DATA, KINOVA_DATA, &KinovaData1,0);

}


int main(int argc, char *argv[]) {

    InitEnvironment();

    ExternalComm extComm;
    // std::cout << std::fixed << std::showpoint;
    extComm.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv,true)); //isSim =  - Simulation true, real false
	extComm.nmpc_obj  = std::unique_ptr<NMPC_A1_KINOVA>(new NMPC_A1_KINOVA(argc,argv));
    extComm.moving_avg_filter = std::unique_ptr<MovingAverageFilter>(new MovingAverageFilter(10,3));
    double kinova_angular_start_pose[4] ={0.0,177.0,325.0,0.0};// {0.0,200.0,325.0,0.0};// {0.0,153.57,60.25,0.0};//{0.0,85.57,147.25,0.0};//{0.0,65.57,147.25,0.0};
    extComm.kinova = std::unique_ptr<KinovaArm>(new KinovaArm(kinova_angular_start_pose));
    extComm.joystick = std::unique_ptr<JoyStick>(new JoyStick());

    Eigen::Matrix<double,2,1>Pstart{0.0,0.0};
    // Agent ID 0
	extComm.loco_obj->setAgentID(0);
    extComm.loco_obj->setPstart(Pstart);
    extComm.nmpc_obj->init();
	extComm.nmpc_obj->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
    extComm.nmpc_obj->setPstart(Pstart);

    
    if (extComm.kinova->getResultComm()!=1)
    {
        
        return 0;

    }

    /*NMPC loop publishing thread*/
    extComm.NMPCThread = CreateRecurrentThreadEx("nmpc_loop", 4, 16700, &ExternalComm::HighLevel, &extComm);
    /*LL loop publishing thread*/
    extComm.CalcThread = CreateRecurrentThreadEx("calc_loop", 1, 1000, &ExternalComm::Calc, &extComm);
    /*Simulation loop publishing thread*/
    extComm.SimExecThread = CreateRecurrentThreadEx("sim_loop", 2, 1000, &ExternalComm::SimExec, &extComm);
    /*Kinova loop publishing thread*/
    extComm.KinovaThread = CreateRecurrentThreadEx("kinova_loop", 6, 16700, &ExternalComm::runKinova, &extComm);
    /*Joystick loop publishing thread*/
    extComm.JoystickThread = CreateRecurrentThreadEx("joystick_loop", 8, 16700, &ExternalComm::JoystickLoop, &extComm);

    while (true)
    {
        sleep(0.1);
    }
    

    // while (true)
	// {
			
    //     // sleep(0.1);
    //     extComm.SimExec();
    //     extComm.HighLevel();
    //     extComm.Calc();
    //     // sim_setup = false;

	// }

    
    return 0;
}


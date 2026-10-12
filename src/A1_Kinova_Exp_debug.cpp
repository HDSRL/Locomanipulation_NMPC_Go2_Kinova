//
// Authror: Basit M. Imran.
// Date : 2024-04-17
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//


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

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LIDAR "rt/utlidar/switch"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);



//Kinova Header
#include <KinovaArm.hpp>


bool rough_terrain_en = 0;
bool obstacle_en = 0;

using std::cout;
using std::cin;

bool first_time0 = 1;
bool first_time1 = 1;
bool first_time2 = 1;
bool first_time3 = 1; 

bool sec_time = 1;
sharedData HLData1;
sharedData LLData1;
sharedData SimData1;
sharedData KinovaData1;
sharedData JoyStickData1;

class ExternalComm {
public:
		// ExternalComm() : udpComp0(8082, "192.168.123.10", 8007, sizeof(LowCmd), sizeof(LowState)){
		ExternalComm() {
            fid = fopen("A1_Kinova_stateData_0003.csv", "w");
			fid_motor = std::fstream("tmp/A1_Kinova_motorData.txt", std::ios::out);

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

			settling = 8*ctrlHz;                   // Settling down
			duration = 4*ctrlHz;                   // Stand up
			stand_settle = 1*ctrlHz; // Settle down after stand up
			loco_start = settling + duration;
			loco_start_kinova = settling + duration; //This is where we can play starting torque control
			trot_start = settling + duration + stand_settle; //This is where we can play starting trot
			
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
	}

	// main thread execution functions
	void Calc();
	void HighLevel();
	void runKinova();
    void holdKinova();
	void exitKinova();
	void JoystickLoop();
	void Init();
	void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
	int queryMotionStatus();
	std::string queryServiceName(std::string form,std::string name);
	
	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<NMPC_A1_KINOVA> nmpc_obj;
    std::unique_ptr<MovingAverageFilter> moving_avg_filter;
	std::unique_ptr<KinovaArm> kinova;
	std::unique_ptr<JoyStick> joystick;
	
    FILE *fid;
	std::fstream fid_motor;

	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};
	double temperature[12] = {0.0};
	

	int StandDuration;
	int SettlingTime;

	FiltStruct_d* jointfilter  = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));
	FiltStruct_f* angfilter    = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
	FiltStruct_f* remotefilter = (FilterStructure_f*)malloc(sizeof(FilterStructure_f));
    FiltStruct_d* linearvelfilter = (FilterStructure_d*)malloc(sizeof(FilterStructure_d));

	xRockerBtnDataStruct remote;

	unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init
	std_msgs::msg::dds_::String_ lidar_switch_msg;

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

	/*LIDAR publisher*/
    ChannelPublisherPtr<std_msgs::msg::dds_::String_> lidar_switch_pub;

	ThreadPtr CalcThread;
    ThreadPtr NMPCThread;
	ThreadPtr JoystickThread;
	ThreadPtr KinovaThread;

	MotionSwitcherClient msc;

	int stop = 0;
	float vel[3] = {0};
	double filtered_vel[3] = {0};
	float pose[6] = {0};
	float ang[2] = {0};
	int actCon[4] = {0};

	float dt = 0.001f;
 // Some parameters in controller0
	size_t loco_kind = STAND;                        // Gait pattern to use
    size_t settling = 8*ctrlHz;                   // Settling down
    size_t duration = 4*ctrlHz;                   // Stand up
	size_t stand_settle = 1*ctrlHz; // Settle down after stand up
    size_t loco_start;        // Start the locomotion pattern
	size_t trot_start;
	size_t loco_start_kinova;        // Start the Kinova Torque control
    timer tset;

	std::vector<double> tau_arm;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF_A1,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF_A1,1);
    int conIndDes[4] = {1,1,1,1};

	float joystick_state[19] = {0};
};

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

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void ExternalComm::Init()
{
    InitLowCmd();

    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&ExternalComm::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    // lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writetorquecmd", UT_CPU_ID_NONE, 1000, &ExternalComm::Calc, this);

	/*LIDAR publisher*/
    lidar_switch_pub.reset(new ChannelPublisher<std_msgs::msg::dds_::String_>(TOPIC_LIDAR));
    lidar_switch_pub->InitChannel();
    lidar_switch_msg.data() = "OFF";  // turn OFF LIDAR
    lidar_switch_pub->Write(lidar_switch_msg);

	/*init MotionSwitcherClient*/
    msc.SetTimeout(10.0f); 
    msc.Init();
    /*Shut down motion control-related service*/
    while(queryMotionStatus())
    {
        std::cout << "Try to deactivate the motion control-related service." << std::endl;
        int32_t ret = msc.ReleaseMode(); 
        if (ret == 0) {
            std::cout << "ReleaseMode succeeded." << std::endl;
        } else {
            std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
        }
        sleep(5);
    }

}

void ExternalComm::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void ExternalComm::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

int ExternalComm::queryMotionStatus()
{
    std::string robotForm,motionName;
    int motionStatus;
    int32_t ret = msc.CheckMode(robotForm,motionName);
    if (ret == 0) {
        std::cout << "CheckMode succeeded." << std::endl;
    } else {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if(motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = queryServiceName(robotForm,motionName);
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string ExternalComm::queryServiceName(std::string form,std::string name)
{
    if(form == "0")
    {
        if(name == "normal" ) return "sport_mode"; 
        if(name == "ai" ) return "ai_sport"; 
        if(name == "advanced" ) return "advanced_sport"; 
    }
    else
    {
        if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
        if(name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}


void ExternalComm::JoystickLoop() {
    // Joystick control loop]
    joystick->getJoyShockState(JoyStickData1.joystick_state);
    updateData(SET_DATA, JOYSTICK_DATA, &JoyStickData1 , 1);
    //Print the right and left stick values
    /* std::cout << "Right Stick: " << joystick_state[0] << ", " << joystick_state[1] << std::endl;
    std::cout << "Left Stick: " << joystick_state[2] << ", " << joystick_state[3] << std::endl;
    //Print the button values D_padup, D_paddown, D_padleft, D_padright
    std::cout << "D_padup: " << joystick_state[4] << ", D_paddown: " << joystick_state[5] << std::endl;
    std::cout << "D_padleft: " << joystick_state[6] << ", D_padright: " << joystick_state[7] << std::endl;
    //Print the button L1, L2, R1, R2
    std::cout << "L1: " << joystick_state[8] << ", L2: " << joystick_state[9] << std::endl;
    std::cout << "R1: " << joystick_state[10] << ", R2: " << joystick_state[11] << std::endl;
    //Print the button cross, circle, square, triangle
    std::cout << "cross: " << joystick_state[12] << ", circle: " << joystick_state[13] << std::endl;
    std::cout << "square: " << joystick_state[14] << ", triangle: " << joystick_state[15] << std::endl;
    //Print the button share, options
    std::cout << "share: " << joystick_state[16] << ", options: " << joystick_state[17] << std::endl;
 */
    
    
   
}



void ExternalComm::HighLevel(){

	auto start = std::chrono::high_resolution_clock::now();

    updateData(GET_DATA, HL_DATA, &HLData1,1);
    // if ( HLData1.runMPC == 1)
	if (true)
        {
            // std::cout << "[Running MPC]" << std::endl;
			/* filtered_vel[0] = HLData1.dq[0];
			filtered_vel[1] = HLData1.dq[1];
			filtered_vel[2] = HLData1.dq[2]; */
			
			// discrete_butter_d(linearvelfilter,&HLData1.dq[0]);

			// std::cout << filtered_vel[0] << "," << filtered_vel[1] << "," << filtered_vel[2] << std::endl;           
            
            // nmpc_obj->updateState(HLData1.q, HLData1.dq, HLData1.ind, HLData1.toePos, HLData1.last_state0, HLData1.domain, HLData1.motiontime);
            nmpc_obj->updateState(HLData1.q, HLData1.dq, HLData1.q_arm, HLData1.dq_arm, HLData1.ind, HLData1.toePos, HLData1.last_state1, HLData1.domain, HLData1.motiontime);
			nmpc_obj->updateReference(HLData1.joystick_state);
			HLData1.runMPC = 0;
                                         
            nmpc_obj->run_NMPC();
            
            HLData1.MPC_data_available = 1;
            HLData1.alpha_COM = nmpc_obj->get_alphaCOM();
            HLData1.alpha_GRF = nmpc_obj->get_alphaGRF();
            HLData1.MPC_sol_ = nmpc_obj->get_MPCsol();
            HLData1.MPC_sol_GRF_ = nmpc_obj->get_MPCsolGRF();
			HLData1.gait = nmpc_obj->get_gait();
			HLData1.reduce_swing_height = nmpc_obj->get_reduce_swing_height();
			// tau_arm = nmpc_obj->get_arm_input_torques();
			tau_arm = nmpc_obj->get_arm_input_torques_PD();
/*             if (HLData1.control_tick <= settling_nmpc)
            {
                tau_arm = {0, 9.94638, -3.12554, 0};
                // tau_arm = {0, 0,0, 0};
            } */
            HLData1.jointTorqueKinova = Eigen::Map<Eigen::Matrix<double,4,1>>(tau_arm.data(), tau_arm.size());
			
			std::vector<double> temp_contacts = nmpc_obj->get_contacts();
		    std::vector<double> temp_Pf = nmpc_obj->get_Pf();
            std::vector<double> temp_GRF = nmpc_obj->get_GRF();  
			std::vector<double> reference_velocity = nmpc_obj->get_reference_velocity();  

            std::copy(temp_contacts.begin(), temp_contacts.end(), HLData1.HL_contacts);
            std::copy(temp_Pf.begin(), temp_Pf.end(), HLData1.HL_feet_vec);
            std::copy(temp_GRF.begin(), temp_GRF.end(), HLData1.HL_GRF);
			std::copy(reference_velocity.begin(), reference_velocity.end(), HLData1.reference_velocity);

            updateData(SET_DATA, HL_DATA, &HLData1,1);
        }
	auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Motiontime: "<< HLData1.motiontime <<" HighLevel Time: " << duration.count() << " us" << std::endl;
}

void ExternalComm::Calc(){

    updateData(GET_DATA, LL_DATA, &LLData1,1);
    
    // ===================================================== //
	// ============== Wireless Remote Stuff  =============== //
	// ===================================================== //
	memcpy(&remote, low_state.wireless_remote().data(), 40);
	if((int)remote.btn.components.B!=0 && (int)remote.btn.components.L2!=0){
		stop = 1;
		low_cmd.reserve() = 0;
	}
	if((int)remote.btn.components.A!=0 && (int)remote.btn.components.L2!=0){
		softFall = true;
		// soft_fall_both = true;
	}
	if( ((int)remote.btn.components.A!=0) && ((int)remote.btn.components.L2 == 0) ){
		// std::cout << "A is pressed!" <<std::endl;
		beginCommand = 1;
		low_cmd.reserve() = 1;
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
		q[i+6] = low_state.motor_state()[i].q();
		dq[i+6] = low_state.motor_state()[i].dq();
		tauEst[i] = low_state.motor_state()[i].tau_est();
		temperature[i] = low_state.motor_state()[i].temperature();
	}
	int footForce[4];
	footForce[0] = low_state.foot_force()[0]; footForce[1] = low_state.foot_force()[1];
	footForce[2] = low_state.foot_force()[2]; footForce[3] = low_state.foot_force()[3];
	// Dump tauEst and temperature to a file and force
	if (fid_motor.is_open()) {
		fid_motor << motiontime << ",";
		for (int i = 0; i < 12; ++i) {
			fid_motor << tauEst[i] << ",";
		}
		for (int i = 0; i < 12; ++i) {
			fid_motor << temperature[i] << ",";
		}
		for (size_t i = 0; i < 4; i++)
		{
			fid_motor << footForce[i] << ",";
		}
		
		fid_motor << "\n";
	} else {
		std::cerr << "Unable to open motor_state.txt for writing." << std::endl;
	}
	discrete_butter_d(jointfilter,&dq[6]);
	

	q[3] = low_state.imu_state().rpy()[0]; q[4] = low_state.imu_state().rpy()[1]; q[5] = low_state.imu_state().rpy()[2];
	dq[3] = low_state.imu_state().gyroscope()[0]; dq[4] = low_state.imu_state().gyroscope()[1]; dq[5] = low_state.imu_state().gyroscope()[2];

	quat_to_XYZ(low_state.imu_state().quaternion()[0],low_state.imu_state().quaternion()[1],low_state.imu_state().quaternion()[2],low_state.imu_state().quaternion()[3],
	            q[3],q[4],q[5]);
	//Offset for Halo 
	q[3]+= -3.0*MY_PI/180.0;
	q[4] += 1.5*MY_PI/180.0;
	q[5] += 0.0*MY_PI/180.0;
	/* //Offset for Willow
	q[3]+=  0*MY_PI/180.0;
	q[4] += 0*MY_PI/180.0;
	q[5] += 0*MY_PI/180.0;
	//dq[5] += 5.0*MY_PI/180.0; */

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(low_state.imu_state().quaternion().data(),R);
	rotMat = R.data();

	// Kinematic Estimation
	static int contactIndex[4] = {1,1,1,1};
	kinEst0(footForce,contactIndex,q,dq,R); // Defined in OtherUtils.hpp




	std::cout << "[LL] Motiontime: " << motiontime << std::endl;
	// ===================================================== //
	// ============= Quad Initialization Time ============== //
	// ===================================================== //
	if ( (motiontime < settling) ){
		for(int i=0; i<12; ++i){
			low_cmd.motor_cmd()[i].dq() = 0.0;
			low_cmd.motor_cmd()[i].kp() = 0.0f;
			low_cmd.motor_cmd()[i].kd() = 0.0f;
			low_cmd.motor_cmd()[i].tau() = 0.0f;
		}
		
	}
	// ===================================================== //
	// ================== LL Controller ==================== //
	// ===================================================== //
	// std::cout << "Controller Executing \n";

    
    // std::cout<<"[Running LL]"<<std::endl;
    // Update the desired torques
    if(motiontime >= settling){ // Settle down
        // LLData1.runMPC = 1;
		if (beginCommand & !softFall){
			if (setup){
				Eigen::Matrix<double, 3, 1> com;
				com(0) = q[0], com(1) = q[1], com(2) = q[2];
				loco_obj->initStandVars(com,(int)duration);
				setup = false;
			}

			// double phaseVar;
            if(motiontime <= loco_start)
            {
                loco_kind = STAND_MPC;
            }
            else if(motiontime <= trot_start)
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

			loco_obj->update_swing_height(LLData1.reduce_swing_height);
            loco_obj->set_MPC_DATA_W_GRF(LLData1.alpha_COM, LLData1.MPC_sol_, LLData1.alpha_GRF, LLData1.MPC_sol_GRF_,LLData1.MPC_data_available);
            loco_obj->update_ref_vel(LLData1.reference_velocity);
			loco_obj->calcZOHTauWithGRF(q,dq,rotMat,footForce,loco_kind,motiontime, &(LLData1.runMPC));
            // loco_obj->calcTauLLonly(q,dq,rotMat,footForce,loco_kind,motiontime, &(LLData1.runMPC));
            
			/* //One step delayed feedback to MPC - done later
			memcpy(LLData1.q,q,18*sizeof(double));
			memcpy(LLData1.dq,dq,18*sizeof(double)); */

            // LLData1.motiontime = motiontime;
            LLData1.MPC_data_available = loco_obj->get_MPC_data_available();
            const int* contactMat = loco_obj->getConDes();
			memcpy(contactIndex,contactMat,4*sizeof(int));
            memcpy(LLData1.ind, contactMat,4*sizeof(int));
            LLData1.toePos = loco_obj->get_toePos();
            LLData1.domain = loco_obj->getDomain();
            LLData1.domain_change = loco_obj->hasDomainChanged();

			// discrete_butter_d(linearvelfilter,&LLData1.dq[0]);
			//Filtering the linear velocity
			// moving_avg_filter->filter(&LLData1.dq[0]);

            // updateData(SET_DATA, LL_DATA, &LLData1,1);


            // Set the command
			for (int i = 0; i < 12; ++i){
				low_cmd.motor_cmd()[i].tau() = loco_obj->ll->tau[i + 6];
				low_cmd.motor_cmd()[i].q()  = loco_obj->ll->q(i + 6);
				low_cmd.motor_cmd()[i].dq() = loco_obj->ll->dq(i + 6);

				low_cmd.motor_cmd()[i].kp() = 10;
				low_cmd.motor_cmd()[i].kd() = 8;
				if(beginPose==true){
					for(int i=0; i<4; ++i){
						if(contactIndex[i]==0){
			//							low_cmd.motor_cmd()[i].tau = 0;
							low_cmd.motor_cmd()[3*i].kp() = 20;
							low_cmd.motor_cmd()[3*i+1].kp() = 20;
							low_cmd.motor_cmd()[3*i+2].kp() = 20;

							low_cmd.motor_cmd()[3*i].kd() = 1;
							low_cmd.motor_cmd()[3*i+1].kd() = 1;
							low_cmd.motor_cmd()[3*i+2].kd() = 1;  
						}
					}	
				}
			}

            // Saturate the command
			float hr_max = 20.0f, hr_min = -20.0f;
			float hp_max = 35.0f, hp_min = -35.0f;
			float kn_max = 43.0f, kn_min = -43.0f;
			for (int i = 0; i < 4; i++){
				low_cmd.motor_cmd()[3 * i + 0].tau() = (low_cmd.motor_cmd()[3 * i + 0].tau() > hr_max) ? hr_max : low_cmd.motor_cmd()[3 * i + 0].tau();
				low_cmd.motor_cmd()[3 * i + 0].tau() = (low_cmd.motor_cmd()[3 * i + 0].tau() < hr_min) ? hr_min : low_cmd.motor_cmd()[3 * i + 0].tau();
				low_cmd.motor_cmd()[3 * i + 1].tau() = (low_cmd.motor_cmd()[3 * i + 1].tau() > hp_max) ? hp_max : low_cmd.motor_cmd()[3 * i + 1].tau();
				low_cmd.motor_cmd()[3 * i + 1].tau() = (low_cmd.motor_cmd()[3 * i + 1].tau() < hp_min) ? hp_min : low_cmd.motor_cmd()[3 * i + 1].tau();
				low_cmd.motor_cmd()[3 * i + 2].tau() = (low_cmd.motor_cmd()[3 * i + 2].tau() > kn_max) ? kn_max : low_cmd.motor_cmd()[3 * i + 2].tau();
				low_cmd.motor_cmd()[3 * i + 2].tau() = (low_cmd.motor_cmd()[3 * i + 2].tau() < kn_min) ? kn_min : low_cmd.motor_cmd()[3 * i + 2].tau();
			}
        }
        else if ( softFall ) 
		{
			for (int i = 0; i < 12; ++i){
				low_cmd.motor_cmd()[i].tau() = 0.0;
				low_cmd.motor_cmd()[i].q()   = 0.0;
				low_cmd.motor_cmd()[i].dq()  = 0.0;

				low_cmd.motor_cmd()[i].kp() = 0;
				low_cmd.motor_cmd()[i].kd() = ( ( (i+1)%3 ) == 0 ) ? 6 : 3;
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
	//One step delayed feedback to MPC
	LLData1.motiontime = motiontime;
	memcpy(LLData1.q,q,18*sizeof(double));
	memcpy(LLData1.dq,dq,18*sizeof(double));
	updateData(SET_DATA, LL_DATA, &LLData1,1);

	motiontime += 1;
    
	low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
    lowcmd_publisher->Write(low_cmd);


}

void ExternalComm::runKinova(){
	
    
    //Get Kinova Data
    updateData(GET_DATA, KINOVA_DATA, &KinovaData1,1);
	// Eigen::Matrix<double, 4, 1> tau_arm = Eigen::Matrix<double, 4, 1>::Zero();
    if (KinovaData1.motiontime >= loco_start_kinova){
		// std::cout << "[Running Kinova Torque]" << std::endl;
        kinova->runTorqueControl(KinovaData1.jointTorqueKinova);
		// kinova->runTorqueControl(tau_arm);
		kinova->sendGripperCommand(KinovaData1.joystick_state);
    }
    

    kinova->getAngularPosition(KinovaData1.q_arm);
    kinova->getAngularVelocity(KinovaData1.dq_arm);
	std::cout << "Kinova Motiontime: "<< KinovaData1.motiontime << std::endl;
    updateData(SET_DATA, KINOVA_DATA, &KinovaData1,1);

}

void ExternalComm::holdKinova(){
    //Get Kinova Data
    updateData(GET_DATA, KINOVA_DATA, &KinovaData1,1);
    
    kinova->holdPosition();

    kinova->getAngularPosition(KinovaData1.q_arm);
    kinova->getAngularVelocity(KinovaData1.dq_arm);
    

    updateData(SET_DATA, KINOVA_DATA, &KinovaData1,1);

}


/* void ExternalComm::exitKinova(){
	kinova->cleanUpKinova();
} */



int main(int argc, char *argv[]) {

    if (argc < 4)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

	ChannelFactory::Instance()->Init(0, argv[4]);

    ExternalComm extComm;
    // std::cout << std::fixed << std::showpoint;
    extComm.loco_obj = std::unique_ptr<LocoWrapper>(new LocoWrapper(argc, argv,false)); //isSim =  - Simulation true, real false
	extComm.nmpc_obj  = std::unique_ptr<NMPC_A1_KINOVA>(new NMPC_A1_KINOVA(argc,argv));
	extComm.moving_avg_filter = std::unique_ptr<MovingAverageFilter>(new MovingAverageFilter(10,3));
    double kinova_angular_start_pose[4] =  {0.0,177.0,325.0,0.0};//{0.0,200.0,325.0,0.0};//{0.0,180.0,270.0,0.0};//{0.0,153.57,60.25,0.0};//{0.0,65.57,147.25,0.0};
    extComm.kinova = std::unique_ptr<KinovaArm>(new KinovaArm(kinova_angular_start_pose));
	extComm.joystick = std::unique_ptr<JoyStick>(new JoyStick());

	if (extComm.kinova->getResultComm()!=1)
    {
        
        return 0;

    }
	
	
	
	Eigen::Matrix<double,2,1>Pstart{0.0,0.0};
    // Agent ID 0
	extComm.loco_obj->setAgentID(0);
    extComm.loco_obj->setPstart(Pstart);
    extComm.nmpc_obj->init();
	extComm.nmpc_obj->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
    extComm.nmpc_obj->setPstart(Pstart);

	extComm.Init(); //Start threads and publishers
    
	/*NMPC loop publishing thread*/
    extComm.NMPCThread = CreateRecurrentThreadEx("nmpc_loop", 4, 16700, &ExternalComm::HighLevel, &extComm);
    /*LL loop publishing thread*/
    extComm.CalcThread = CreateRecurrentThreadEx("calc_loop", 1, 1000, &ExternalComm::Calc, &extComm);
	/*Kinova loop publishing thread*/
	extComm.KinovaThread = CreateRecurrentThreadEx("kinova_loop", 5, 16700, &ExternalComm::runKinova, &extComm);
	/*Joystick loop publishing thread*/
	extComm.JoystickThread = CreateRecurrentThreadEx("joystick_loop", 8, 16700, &ExternalComm::JoystickLoop, &extComm);
	

    while ( (extComm.stop==0) ){
		sleep(0.1);
	};
	fclose(extComm.fid);
	return 0;
}


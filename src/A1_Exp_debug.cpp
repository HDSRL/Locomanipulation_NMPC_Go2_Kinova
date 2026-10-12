//
// Authror: Basit M. Imran.
// Date : 2024-04-17
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "nmpc.hpp"
#include "OtherUtils.hpp"
#include "timer.h"
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
//HDSRL header
#include "LocoWrapper.hpp"
#include "shared_structs.hpp"

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

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LIDAR "rt/utlidar/switch"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

bool rough_terrain_en = 0;
bool obstacle_en = 0;

using std::cout;
using std::cin;

bool first_time0 = 1;
bool first_time1 = 1;
bool first_time2 = 1;
bool first_time3 = 1; 

bool sec_time = 1;
sharedData HLData0;
sharedData LLData0;
sharedData SimData0;

class ExternalComm {
public:
		// ExternalComm() : udpComp0(8082, "192.168.123.10", 8007, sizeof(LowCmd), sizeof(LowState)){
		ExternalComm() {
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

			//4 Hz 
            double av[5] = { 1.000000000, -3.934325821, 5.805125421,-3.807232457, 0.936433243}; 
			double bv[5] = {0.000000024, 0.000000097, 0.000000145,0.000000097, 0.000000024}; 
            populate_filter_d(linearvelfilter, av, bv, 5, 3);

			loco_start = settling + duration;

			
		}
	
		

	virtual ~ExternalComm(){
		clear_filter_d(jointfilter);
		clear_filter_f(remotefilter);
		clear_filter_f(angfilter);
		clear_filter_d(linearvelfilter);

		nmpc_obj->logData();
        nmpc_obj.reset();
        loco_obj.reset();
        
	}

	// main thread execution functions
	void Calc();
	void HighLevel();
	void Init();
	void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
	int queryMotionStatus();
	std::string queryServiceName(std::string form,std::string name);
	
	std::unique_ptr<LocoWrapper> loco_obj;
	std::unique_ptr<NMPC> nmpc_obj;
    std::unique_ptr<MovingAverageFilter> moving_avg_filter;
    FILE *fid;

	long motiontime = 0;
	bool softFall = false;
	bool beginCommand = false;
	bool beginPose = false;
	bool setup = true;
	double q[18] = {0.0};
	double dq[18] = {0.0};
	double tauEst[12] = {0.0};
	

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
    size_t duration = 10*ctrlHz;                   // Stand up
    size_t loco_start;        // Start the locomotion pattern
    timer tset;

    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    Eigen::VectorXd 
	jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    int conIndDes[4] = {1,1,1,1};


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


void ExternalComm::HighLevel(){

    updateData(GET_DATA, HL_DATA, &HLData0,0);
    // if ( HLData0.runMPC == 1)
	if ( true)
        {
            // std::cout << "[Running MPC]" << std::endl;
			/* filtered_vel[0] = HLData0.dq[0];
			filtered_vel[1] = HLData0.dq[1];
			filtered_vel[2] = HLData0.dq[2]; */
			
			// discrete_butter_d(linearvelfilter,&HLData0.dq[0]);

			// std::cout << filtered_vel[0] << "," << filtered_vel[1] << "," << filtered_vel[2] << std::endl;           
            
            nmpc_obj->updateState(HLData0.q, HLData0.dq, HLData0.ind, HLData0.toePos, HLData0.last_state0, HLData0.domain, HLData0.motiontime);
            HLData0.runMPC = 0;
                                         
            nmpc_obj->run_NMPC();
            
            HLData0.MPC_data_available = 1;
            HLData0.alpha_COM = nmpc_obj->get_alphaCOM();
            HLData0.alpha_GRF = nmpc_obj->get_alphaGRF();
            HLData0.MPC_sol_ = nmpc_obj->get_MPCsol();
            HLData0.MPC_sol_GRF_ = nmpc_obj->get_MPCsolGRF();
            
            updateData(SET_DATA, HL_DATA, &HLData0,0);
        }
}

void ExternalComm::Calc(){

    updateData(GET_DATA, LL_DATA, &LLData0,0);
    
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
	}
	discrete_butter_d(jointfilter,&dq[6]);

	q[3] = low_state.imu_state().rpy()[0]; q[4] = low_state.imu_state().rpy()[1]; q[5] = low_state.imu_state().rpy()[2];
	dq[3] = low_state.imu_state().gyroscope()[0]; dq[4] = low_state.imu_state().gyroscope()[1]; dq[5] = low_state.imu_state().gyroscope()[2];

	quat_to_XYZ(low_state.imu_state().quaternion()[0],low_state.imu_state().quaternion()[1],low_state.imu_state().quaternion()[2],low_state.imu_state().quaternion()[3],
	            q[3],q[4],q[5]);
	//Offset for Halo 
	/* q[3]+= -3.0*MY_PI/180.0;
	q[4] += 1.5*MY_PI/180.0;
	q[5] += 0.0*MY_PI/180.0; */
	/* //Offset for Willow
	q[3]+=  0*MY_PI/180.0;
	q[4] += 0*MY_PI/180.0;
	q[5] += 0*MY_PI/180.0;
	//dq[5] += 5.0*MY_PI/180.0; */
	//Offset for Tom 
	q[3]+= -3.0*MY_PI/180.0;
	q[4] += 0.0*MY_PI/180.0;
	q[5] += 0.0*MY_PI/180.0;

	// std::cout << "q[3]: " << q[3] << ", q[4]: " << q[4] << ", q[5]: " << q[5] << std::endl;

	double *rotMat;
	Eigen::Matrix3d R;
	quat_to_R(low_state.imu_state().quaternion().data(),R);
	rotMat = R.data();

	// Kinematic Estimation
	int footForce[4];
	footForce[0] = low_state.foot_force()[0]; footForce[1] = low_state.foot_force()[1];
	footForce[2] = low_state.foot_force()[2]; footForce[3] = low_state.foot_force()[3];
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
        // LLData0.runMPC = 1;
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
            else if(motiontime > loco_start)
            {
                loco_kind = TROT;
            }

       
            loco_obj->set_MPC_DATA_W_GRF(LLData0.alpha_COM, LLData0.MPC_sol_, LLData0.alpha_GRF, LLData0.MPC_sol_GRF_,LLData0.MPC_data_available);
            loco_obj->calcZOHTauWithGRF(q,dq,rotMat,footForce,loco_kind,motiontime, &(LLData0.runMPC));
            // loco_obj->calcTauLLonly(q,dq,rotMat,footForce,loco_kind,motiontime, &(LLData0.runMPC));
            
			//One step delayed feedback to MPC
			memcpy(LLData0.q,q,18*sizeof(double));
			memcpy(LLData0.dq,dq,18*sizeof(double));

            LLData0.motiontime = motiontime;
            LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
            const int* contactMat = loco_obj->getConDes();
			memcpy(contactIndex,contactMat,4*sizeof(int));
            memcpy(LLData0.ind, contactMat,4*sizeof(int));
            LLData0.toePos = loco_obj->get_toePos();
            LLData0.domain = loco_obj->getDomain();
            LLData0.domain_change = loco_obj->hasDomainChanged();

			// discrete_butter_d(linearvelfilter,&LLData0.dq[0]);
			moving_avg_filter->filter(&LLData0.dq[0]);
	

            updateData(SET_DATA, LL_DATA, &LLData0,0);


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
			float hr_max = 10.0f, hr_min = -10.0f;
			float hp_max = 30.0f, hp_min = -30.0f;
			float kn_max = 33.0f, kn_min = -33.0f;
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
	motiontime += 1;

	low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    
    lowcmd_publisher->Write(low_cmd);


}



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
	extComm.nmpc_obj  = std::unique_ptr<NMPC>(new NMPC(argc,argv));
	extComm.moving_avg_filter = std::unique_ptr<MovingAverageFilter>(new MovingAverageFilter(100,3));
    Eigen::Matrix<double,2,1>Pstart{0.0,0.0};
    // Agent ID 0
	extComm.loco_obj->setAgentID(0);
    extComm.loco_obj->setPstart(Pstart);
    extComm.nmpc_obj->init();
	extComm.nmpc_obj->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
    extComm.nmpc_obj->setPstart(Pstart);
	
	extComm.Init(); //Start threads and publishers

    /*NMPC loop publishing thread*/
    extComm.NMPCThread = CreateRecurrentThreadEx("nmpc_loop", 2, 10000, &ExternalComm::HighLevel, &extComm);
    /*LL loop publishing thread*/
    extComm.CalcThread = CreateRecurrentThreadEx("calc_loop", 1, 1000, &ExternalComm::Calc, &extComm);


    while ( (extComm.stop==0) ){
		sleep(0.1);
	};
	fclose(extComm.fid);
	return 0;
}


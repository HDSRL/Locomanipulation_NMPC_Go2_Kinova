#ifndef SHARED_DATA
#define SHARED_DATA

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Sparse"

#include "mutex"
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <vector>
#define SET_DATA 1
#define GET_DATA 0
#define HL_DATA 1
#define LL_DATA 0
#define SIM_DATA 2
#define KINOVA_DATA 3
#define JOYSTICK_DATA 4
#define HORIZON_SIM 15

boost::mutex mtx0;
boost::mutex mtx1;


struct sharedData
{
	int resetRun = 0;

	// LL sets, used by HL
	double q[18] = {0};
	double dq[18] = {0};

	double q_arm[4] = {0};
	double dq_arm[4] = {0};

	//Sim sets, used by LL,HL
	// Added for mutli-threaded simulation
	double rotMatrixDouble[9] = {0};
	// float quaternion[4] = {0};
	int footForce[4] = {0};
	Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(18,1); 
	Eigen::VectorXd jointTorqueA1 = Eigen::MatrixXd::Zero(18,1);
	Eigen::VectorXd jointTorqueKinova = Eigen::MatrixXd::Zero(4,1);
    //LL additional feedbacks
	double q_LL[18] = {0};
	double dq_LL[18] = {0};
	int ind_LL[4] = {1,1,1,1};
	long motiontime = 0;
	//////////////////////////////////////

	//HL sets, used for plotting
	double HL_GRF[12]={0};
	double HL_feet_vec[12]={0};
	double HL_contacts[4]={0};
	double reference_velocity[3] = {0}; // Used for plotting

	int ind[4] = {1,1,1,1};
	int domain = 0;
	int control_tick = 0; 
	int domain_change = 0;
	Eigen::Matrix<double, 3, 4> toePos = Eigen::MatrixXd::Zero(3,4);

	// HL sets, used by LL
	Eigen::MatrixXd alpha_COM = Eigen::MatrixXd::Zero(4, 5);
	Eigen::MatrixXd MPC_sol_;
	Eigen::MatrixXd alpha_GRF;
	Eigen::MatrixXd MPC_sol_GRF_;
	bool gait = 0; // Used to change the gait pattern
	bool reduce_swing_height = 0; // Used to reduce the swing height of the LL

	// Both set, both use
	Eigen::Vector4d last_state0 = Eigen::MatrixXd::Zero(4,1);
	Eigen::Vector4d last_state1 = Eigen::MatrixXd::Zero(4,1); 
	bool runMPC = 1;
	bool MPC_data_available = 0;

	//Joystick 
	float joystick_state[19] = {0};
	
	

};

sharedData data0;
sharedData data1;
sharedData data2;
sharedData data3;

void updateData0(int setget, int highlow, sharedData* newData)	// SETGET, HIGHLOW,  sharedData* newData
{ 
	//Documentation
	
	// set = 1, get = 0
	// high = 1, low = 0
	// boost::lock_guard<boost::mutex> guard(mtx0);

	if (setget == SET_DATA)
	{
		// SET DATA
		if (highlow == HL_DATA)
		{
			data0.alpha_COM = newData->alpha_COM;
			data0.MPC_sol_ = newData->MPC_sol_;
			data0.alpha_GRF = newData->alpha_GRF;
			data0.MPC_sol_GRF_ = newData->MPC_sol_GRF_;
			data0.domain = newData->domain;
			data0.domain_change = newData->domain_change;
			data0.MPC_data_available = newData->MPC_data_available;
			data0.runMPC = newData->runMPC;
			data0.resetRun = newData->resetRun;
			data0.last_state0 = newData->last_state0;
			//For multithreaded Simulation
			memcpy(data0.HL_contacts,newData->HL_contacts,4*sizeof(double));
			memcpy(data0.HL_feet_vec,newData->HL_feet_vec,12*sizeof(double));
			memcpy(data0.HL_GRF,newData->HL_GRF,12*sizeof(double));
		}
		else if (highlow == LL_DATA)
		{
			memcpy(data0.q,newData->q,18*sizeof(double)); 
			memcpy(data0.dq,newData->dq,18*sizeof(double));
			memcpy(data0.ind,newData->ind,4*sizeof(int));
			data0.domain = newData->domain;
			data0.domain_change = newData->domain_change;
			data0.motiontime = newData->motiontime;
			data0.toePos = newData->toePos;
			if(data0.resetRun==-1 && newData->resetRun==-2){
				data0.resetRun = 0;
			}
			if (data0.resetRun==0){
				data0.runMPC = newData->runMPC;
				data0.MPC_data_available = newData->MPC_data_available;
			}
			//For multithreaded Simulation
			data0.jointTorqueFF = newData->jointTorqueFF;
			memcpy(data0.ind_LL,newData->ind_LL,4*sizeof(int));
		}
		else if (highlow == SIM_DATA)	
		{
			// memcpy(data0.quaternion,newData->quaternion,4*sizeof(float));
			memcpy(data0.footForce,newData->footForce,4*sizeof(int));
			memcpy(data0.q_LL,newData->q_LL,18*sizeof(double));
			memcpy(data0.dq_LL,newData->dq_LL,18*sizeof(double));
			memcpy(data0.rotMatrixDouble,newData->rotMatrixDouble,9*sizeof(double));
			data0.control_tick = newData->control_tick;

			
		}

		else if (highlow == KINOVA_DATA)
		{
			memcpy(data0.q_arm,newData->q_arm,4*sizeof(double));
			memcpy(data0.dq_arm,newData->dq_arm,4*sizeof(double));
			data0.control_tick = newData->control_tick;
			data0.motiontime = newData->motiontime;
		}
		
	}
	else	// GET DATA
	{
		if (highlow == HL_DATA)
		{
			memcpy(newData->q,data0.q,18*sizeof(double)); 
			memcpy(newData->dq,data0.dq,18*sizeof(double));
			memcpy(newData->ind,data0.ind,4*sizeof(int));
			newData->MPC_data_available = data0.MPC_data_available;
			newData->runMPC = data0.runMPC;
			newData->motiontime = data0.motiontime;
			newData->resetRun = data0.resetRun;
			newData->toePos = data0.toePos;
			newData->domain = data0.domain;					// Update domain from LL struct to HL struct
			newData->domain_change = data0.domain_change;					// Update domain from LL struct to HL struct
			newData->control_tick = data0.control_tick;
			newData->motiontime = data0.motiontime;
			newData->last_state1 = data1.last_state1;
			//For multithreaded Simulation
			memcpy(newData->q_LL,data0.q_LL,18*sizeof(double)); 
			memcpy(newData->dq_LL,data0.dq_LL,18*sizeof(double));
			memcpy(newData->ind_LL,data0.ind_LL,4*sizeof(int));
			memcpy(newData->q_arm,data0.q_arm,4*sizeof(double));
			memcpy(newData->dq_arm,data0.dq_arm,4*sizeof(double));
			
		}
		else if(highlow == LL_DATA)
		{
			newData->alpha_COM = data0.alpha_COM;
			newData->MPC_sol_ = data0.MPC_sol_;
			newData->alpha_GRF = data0.alpha_GRF;
			newData->MPC_sol_GRF_ = data0.MPC_sol_GRF_;
			newData->MPC_data_available = data0.MPC_data_available;
			newData->runMPC = data0.runMPC;
			newData->resetRun = data0.resetRun;
			newData->domain = data0.domain;
			newData->domain_change = data0.domain_change;
			newData->control_tick = data0.control_tick;
			newData->last_state1 = data1.last_state1;
			//For multithreaded Simulation
			memcpy(newData->rotMatrixDouble,data0.rotMatrixDouble,9*sizeof(double));
			memcpy(newData->footForce,data0.footForce,4*sizeof(int));
			// memcpy(newData->quaternion,data0.quaternion,4*sizeof(float));
			memcpy(newData->q_LL,data0.q_LL,18*sizeof(double));
			memcpy(newData->dq_LL,data0.dq_LL,18*sizeof(double));
			// memcpy(newData->ind_LL,data0.ind_LL,4*sizeof(int));
		}
		else if(highlow == SIM_DATA)
		{
			newData->jointTorqueFF = data0.jointTorqueFF;
			memcpy(newData->HL_contacts,data0.HL_contacts,4*sizeof(double));
			memcpy(newData->HL_feet_vec,data0.HL_feet_vec,12*sizeof(double));
			memcpy(newData->HL_GRF,data0.HL_GRF,12*sizeof(double));
			memcpy(newData->ind_LL,data0.ind_LL,4*sizeof(int));
		}
		else if(highlow == KINOVA_DATA)
		{
			memcpy(newData->q_arm,data0.q_arm,4*sizeof(double));
			memcpy(newData->dq_arm,data0.dq_arm,4*sizeof(double));
			newData->control_tick = data0.control_tick;
			newData->motiontime = data0.motiontime;
			newData->jointTorqueKinova = data0.jointTorqueKinova;
		}
	}
}

void updateData1(int setget, int highlow, sharedData* newData)	// SETGET, HIGHLOW,  sharedData* newData
{ 
	// set = 1, get = 0
	// high = 1, low = 0
	// boost::lock_guard<boost::mutex> guard(mtx1);

	if (setget == SET_DATA)
	{
		// SET DATA
		if (highlow == HL_DATA)
		{
			data1.alpha_COM = newData->alpha_COM;
			data1.MPC_sol_ = newData->MPC_sol_;
			data1.alpha_GRF = newData->alpha_GRF;
			data1.MPC_sol_GRF_ = newData->MPC_sol_GRF_;
			data1.MPC_data_available = newData->MPC_data_available;
			data1.runMPC = newData->runMPC;
			data1.resetRun = newData->resetRun;
			data1.last_state0 = newData->last_state0;
			//For multithreaded Simulation
			memcpy(data1.HL_contacts,newData->HL_contacts,4*sizeof(double));
			memcpy(data1.HL_feet_vec,newData->HL_feet_vec,12*sizeof(double));
			memcpy(data1.HL_GRF,newData->HL_GRF,12*sizeof(double));
			//Arm
			data1.jointTorqueKinova = newData->jointTorqueKinova;
			
			//reference velocity
			memcpy(data1.reference_velocity,newData->reference_velocity,3*sizeof(double));
			//gait
			data1.gait = newData->gait;
			data1.reduce_swing_height = newData->reduce_swing_height;
		}
		else if (highlow == LL_DATA)
		{
			memcpy(data1.q,newData->q,18*sizeof(double)); 
			memcpy(data1.dq,newData->dq,18*sizeof(double));
			memcpy(data1.ind,newData->ind,4*sizeof(int));
			data1.domain = newData->domain;
			data1.domain_change = newData->domain_change;
			data1.motiontime = newData->motiontime;
			data1.toePos = newData->toePos;
			if(data1.resetRun==-1 && newData->resetRun==-2){
				data1.resetRun = 0;
			}
			if (data1.resetRun==0){
				data1.runMPC = newData->runMPC;
				data1.MPC_data_available = newData->MPC_data_available;
			}
			//For multithreaded Simulation
			data1.jointTorqueFF = newData->jointTorqueFF;
			data1.jointTorqueA1 = newData->jointTorqueA1;
			memcpy(data1.ind_LL,newData->ind_LL,4*sizeof(int));
			
		}


		else if (highlow == SIM_DATA)	
		{
			// memcpy(data0.quaternion,newData->quaternion,4*sizeof(float));
			memcpy(data1.footForce,newData->footForce,4*sizeof(int));
			memcpy(data1.q_LL,newData->q_LL,18*sizeof(double));
			memcpy(data1.q_arm,newData->q_arm,4*sizeof(double));
			memcpy(data1.dq_LL,newData->dq_LL,18*sizeof(double));
			memcpy(data1.dq_arm,newData->dq_arm,4*sizeof(double));
			memcpy(data1.rotMatrixDouble,newData->rotMatrixDouble,9*sizeof(double));
			data1.control_tick = newData->control_tick;

			
		}

		else if (highlow == KINOVA_DATA)
		{
			memcpy(data1.q_arm,newData->q_arm,4*sizeof(double));
			memcpy(data1.dq_arm,newData->dq_arm,4*sizeof(double));
			// data1.control_tick = newData->control_tick;
			// data1.motiontime = newData->motiontime;
		}
		else if (highlow == JOYSTICK_DATA)
		{
			memcpy(data1.joystick_state,newData->joystick_state,sizeof(data1.joystick_state));
		}
	}
	else	// GET DATA
	{
		if (highlow == HL_DATA)
		{
			memcpy(newData->q,data1.q,18*sizeof(double)); 
			memcpy(newData->dq,data1.dq,18*sizeof(double));
			memcpy(newData->ind,data1.ind,4*sizeof(int));
			newData->MPC_data_available = data1.MPC_data_available;
			newData->runMPC = data1.runMPC;
			newData->motiontime = data1.motiontime;
			newData->resetRun = data1.resetRun;
			newData->toePos = data1.toePos;
			newData->domain = data1.domain;					// Update domain from LL struct to HL struct
			newData->domain_change = data1.domain_change;					// Update domain from LL struct to HL struct
			newData->control_tick = data1.control_tick;
			newData->motiontime = data1.motiontime;
			newData->last_state1 = data1.last_state1;
			//For multithreaded Simulation
			memcpy(newData->q_LL,data1.q_LL,18*sizeof(double)); 
			memcpy(newData->dq_LL,data1.dq_LL,18*sizeof(double));
			memcpy(newData->ind_LL,data1.ind_LL,4*sizeof(int));
			memcpy(newData->q_arm,data1.q_arm,4*sizeof(double));
			memcpy(newData->dq_arm,data1.dq_arm,4*sizeof(double));
			memcpy(newData->joystick_state,data1.joystick_state,sizeof(data1.joystick_state));
			
		}
		else if(highlow == LL_DATA)
		{
			newData->alpha_COM = data1.alpha_COM;
			newData->MPC_sol_ = data1.MPC_sol_;
			newData->alpha_GRF = data1.alpha_GRF;
			newData->MPC_sol_GRF_ = data1.MPC_sol_GRF_;
			newData->MPC_data_available = data1.MPC_data_available;
			newData->runMPC = data1.runMPC;
			newData->resetRun = data1.resetRun;
			newData->domain = data1.domain;
			newData->domain_change = data1.domain_change;
			newData->control_tick = data1.control_tick;
			newData->last_state1 = data1.last_state1;
			//For multithreaded Simulation
			memcpy(newData->rotMatrixDouble,data1.rotMatrixDouble,9*sizeof(double));
			memcpy(newData->footForce,data1.footForce,4*sizeof(int));
			// memcpy(newData->quaternion,data0.quaternion,4*sizeof(float));
			memcpy(newData->q_LL,data1.q_LL,18*sizeof(double));
			memcpy(newData->dq_LL,data1.dq_LL,18*sizeof(double));
			// memcpy(newData->ind_LL,data0.ind_LL,4*sizeof(int));
			//reference velocity
			memcpy(newData->reference_velocity,data1.reference_velocity,3*sizeof(double));
			//gait
			newData->gait = data1.gait;
			newData->reduce_swing_height = data1.reduce_swing_height;
		}

		else if(highlow == SIM_DATA)
		{
			newData->jointTorqueFF = data1.jointTorqueFF;
			newData->jointTorqueA1 = data1.jointTorqueA1;
			newData->jointTorqueKinova = data1.jointTorqueKinova;
			memcpy(newData->HL_contacts,data1.HL_contacts,4*sizeof(double));
			memcpy(newData->HL_feet_vec,data1.HL_feet_vec,12*sizeof(double));
			memcpy(newData->HL_GRF,data1.HL_GRF,12*sizeof(double));
			memcpy(newData->ind_LL,data1.ind_LL,4*sizeof(int));
		}

		else if(highlow == KINOVA_DATA)
		{
			// memcpy(newData->q_arm,data1.q_arm,4*sizeof(double));
			// memcpy(newData->dq_arm,data1.dq_arm,4*sizeof(double));
			newData->control_tick = data1.control_tick;
			newData->motiontime = data1.motiontime;
			newData->jointTorqueKinova = data1.jointTorqueKinova;
			memcpy(newData->joystick_state,data1.joystick_state,sizeof(data1.joystick_state));

		}
	}
}

void updateData2(int setget, int highlow, sharedData* newData)	// SETGET, HIGHLOW,  sharedData* newData
{ 
	// set = 1, get = 0
	// high = 1, low = 0
	// boost::lock_guard<boost::mutex> guard(mtx0);

	if (setget == SET_DATA)
	{
		// SET DATA
		if (highlow == HL_DATA)
		{
			data2.alpha_COM = newData->alpha_COM;
			data2.MPC_sol_ = newData->MPC_sol_;
			data2.domain = newData->domain;
			data2.MPC_data_available = newData->MPC_data_available;
			data2.runMPC = newData->runMPC;
			data2.resetRun = newData->resetRun;
			data2.last_state0 = newData->last_state0;
			// std::cout << "data0.last_state0: " << data0.last_state0.transpose(); std::cin.get();
		}
		else 
		{
			memcpy(data2.q,newData->q,18*sizeof(double)); 
			memcpy(data2.dq,newData->dq,18*sizeof(double));
			memcpy(data2.ind,newData->ind,4*sizeof(int));
			data2.domain = newData->domain;
			data2.toePos = newData->toePos;
			if(data2.resetRun==-1 && newData->resetRun==-2){
				data2.resetRun = 0;
			}
			if (data2.resetRun==0){
				data2.runMPC = newData->runMPC;
				data2.MPC_data_available = newData->MPC_data_available;
			}
		}
	}
	else	// GET DATA
	{
		if (highlow == HL_DATA)
		{
			newData->domain = data2.domain;
			memcpy(newData->q,data2.q,18*sizeof(double)); 
			memcpy(newData->dq,data2.dq,18*sizeof(double));
			memcpy(newData->ind,data2.ind,4*sizeof(int));
			newData->MPC_data_available = data2.MPC_data_available;
			newData->runMPC = data2.runMPC;
			newData->resetRun = data2.resetRun;
			newData->toePos = data2.toePos;
			newData->last_state1 = data3.last_state1;
		}
		else
		{
			newData->alpha_COM = data2.alpha_COM;
			newData->MPC_sol_ = data2.MPC_sol_;
			newData->MPC_data_available = data2.MPC_data_available;
			newData->runMPC = data2.runMPC;
			newData->resetRun = data2.resetRun;
			newData->domain = data2.domain;
			newData->last_state1 = data3.last_state1;
		}
	}
}

void updateData3(int setget, int highlow, sharedData* newData)	// SETGET, HIGHLOW,  sharedData* newData
{ 
	// set = 1, get = 0
	// high = 1, low = 0
	// boost::lock_guard<boost::mutex> guard(mtx1);

	if (setget == SET_DATA)
	{
		// SET DATA
		if (highlow == HL_DATA)
		{
			data3.alpha_COM = newData->alpha_COM;
			data3.MPC_sol_ = newData->MPC_sol_;
			data3.domain = newData->domain;
			data3.MPC_data_available = newData->MPC_data_available;
			data3.runMPC = newData->runMPC;
			data3.resetRun = newData->resetRun;
			data3.last_state1 = newData->last_state1;
		}
		else 
		{
			memcpy(data3.q,newData->q,18*sizeof(double)); 
			memcpy(data3.dq,newData->dq,18*sizeof(double));
			memcpy(data3.ind,newData->ind,4*sizeof(int));
			data3.domain = newData->domain;
			data3.toePos = newData->toePos;
			if(data3.resetRun==-1 && newData->resetRun==-2){
				data3.resetRun = 0;
			}
			if (data3.resetRun==0){
				data3.runMPC = newData->runMPC;
				data3.MPC_data_available = newData->MPC_data_available;
			}
		}
	}
	else	// GET DATA
	{
		if (highlow == HL_DATA)
		{
			newData->domain = data3.domain;
			memcpy(newData->q,data3.q,18*sizeof(double)); 
			memcpy(newData->dq,data3.dq,18*sizeof(double));
			memcpy(newData->ind,data3.ind,4*sizeof(int));
			newData->MPC_data_available = data3.MPC_data_available;
			newData->runMPC = data3.runMPC;
			newData->resetRun = data3.resetRun;
			newData->toePos = data3.toePos;
			newData->last_state0 = data2.last_state0;
			// std::cout << "newData->last_state0: " << newData->last_state0.transpose(); std::cin.get();
		}
		else
		{
			newData->alpha_COM = data3.alpha_COM;
			newData->MPC_sol_ = data3.MPC_sol_;
			newData->MPC_data_available = data3.MPC_data_available;
			newData->runMPC = data3.runMPC;
			newData->resetRun = data3.resetRun;
			newData->domain = data3.domain;
			newData->last_state0 = data2.last_state0;
		}
	}
}

void updateData(int setget, int highlow, sharedData* newData, int agent_number)
{
	boost::lock_guard<boost::mutex> guard(mtx0);

	if (agent_number == 0)
	{
		updateData0(setget, highlow, newData);
	}
	else if (agent_number == 1)
	{
		updateData1(setget, highlow, newData);
	}
	else if (agent_number == 2)
	{
		updateData2(setget, highlow, newData);
	}
	else if (agent_number == 3)
	{
		updateData3(setget, highlow, newData);
	}

}

void backupData(sharedData& copy_to, const sharedData& copy_from) {
    // Use a lock guard to ensure thread safety if necessary
    // boost::lock_guard<boost::mutex> guard(your_mutex);

    // Copying all fields from copy_from to copy_to
    copy_to.resetRun = copy_from.resetRun;
    
    // For arrays, use std::copy
    std::copy(std::begin(copy_from.q), std::end(copy_from.q), std::begin(copy_to.q));
    std::copy(std::begin(copy_from.dq), std::end(copy_from.dq), std::begin(copy_to.dq));
    std::copy(std::begin(copy_from.ind), std::end(copy_from.ind), std::begin(copy_to.ind));
    
    copy_to.domain = copy_from.domain;
    copy_to.toePos = copy_from.toePos;

    // For Eigen matrices, you can simply use assignment as they support deep copy by default
    copy_to.alpha_COM = copy_from.alpha_COM;
    copy_to.MPC_sol_ = copy_from.MPC_sol_;
    copy_to.last_state0 = copy_from.last_state0;
    copy_to.last_state1 = copy_from.last_state1;

    copy_to.runMPC = copy_from.runMPC;
    copy_to.MPC_data_available = copy_from.MPC_data_available;
}


#endif

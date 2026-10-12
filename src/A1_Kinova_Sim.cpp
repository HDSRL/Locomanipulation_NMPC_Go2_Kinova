//
// Author: Basit M. Imran.
// Modified: Ruturaj Sudhir Sambhus 
// Date : 2024-10-17
// Copyright (c) Hybrid Dynamic Systems and Robot Locomotion Lab, Virginia Tech
//

#include "raisim/OgreVis.hpp"
#include "randyImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include "helper2.hpp"
#include "MPC_dist.hpp"
#include "a1_kinova_nmpc.hpp"
#include "OtherUtils.hpp"
#include "timer.h"
#include <cstdlib> // For rand() and srand()
#include <ctime>   // For time() 
#include <fstream>
#include <iostream>
#include <thread>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <string>
#include <casadi/casadi.hpp>
//HDSRL header
#include "LocoWrapper.hpp"
#include "shared_structs.hpp"

// For joystick


bool rough_terrain_en = 1;
bool obstacle_en = 0;
bool push_box_en = 0;
bool disturbance_en = 0;
bool payload_en = 0;
bool gripper_grasp_en = 0;

using std::cout;
using std::cin;

bool first_time0 = 1;
bool first_time1 = 1;
bool first_time2 = 1;
bool first_time3 = 1; 

bool sec_time = 1;
sharedData HLData0;
sharedData LLData0;

sharedData HLData1;
sharedData LLData1;

sharedData HLData2;
sharedData LLData2;

sharedData HLData3;
sharedData LLData3;

sharedData HLData0_Backup, LLData0_Backup;
sharedData HLData1_Backup, LLData1_Backup;
sharedData HLData2_Backup, LLData2_Backup;
sharedData HLData3_Backup, LLData3_Backup;

std::string getCurrentTimestamp() {
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
        std::cerr << "Unable to open file" << std::endl;
    }
}




void setupCallback() {
    raisim::OgreVis *vis = raisim::OgreVis::get();

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

void disturbance(std::vector<raisim::ArticulatedSystem *> A1,std::map<std::string, raisim::VisualObject> &objList,size_t start,size_t stop, size_t ctrlTick, double force_mag, std::string axis){
    Eigen::VectorXd pos = Eigen::MatrixXd::Zero(TOTAL_DOF+1,1);
    Eigen::VectorXd vel = Eigen::MatrixXd::Zero(TOTAL_DOF,1);
    A1.back()->getState(pos, vel);
    
    // ====================== External Forces ====================== //
    raisim::Vec<3> extForce;
    raisim::Mat<3,3> rot;
    raisim::Vec<3> dir;
    if (ctrlTick>=start && ctrlTick<stop){
        if (axis == "bkwd") {extForce = {-force_mag,0,0};} // Fwd
        else if (axis == "lat") {extForce = {0,-force_mag,0};} // Lat
        else if (axis == "up") {extForce = {0,0,-force_mag};} // Up
        else if(axis == "down") {extForce = {0,0,force_mag};} // Down
        else if (axis == "bkwdlat") {extForce = {-force_mag,-force_mag,0};} // FwdLat
        else if (axis == "bkwdup") {extForce = {-force_mag,0,-force_mag};} // FwdUp
        else if (axis == "bkwddown") {extForce = {-force_mag,0,force_mag};} // FwdDown
        else if (axis == "latup") {extForce = {0,-force_mag,-force_mag};} // LatUp
        else if (axis == "latdown") {extForce = {0,-force_mag,force_mag};} // LatDown
        else if (axis == "bkwdlatup") {extForce = {-force_mag,-force_mag,-force_mag};} // FwdLatUp
        else if (axis == "bkwdlatdown") {extForce = {-force_mag,-force_mag,force_mag};} // FwdLatDown
        else if (axis == "latupdown") {extForce = {0,-force_mag,-force_mag};} // LatUpDown
        else if (axis == "bkwdlatupdown") {extForce = {-force_mag,-force_mag,-force_mag};} // FwdLatUpDown
        else if(axis == "all") {extForce = {-force_mag,-force_mag,-force_mag};} // All
        else if(axis=="none") {extForce = {0,0,0};} // None
        else
        {
            std::cerr << "Error: Invalid axis specified for disturbance." << std::endl;
            return;
        }
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

void plotGRFs(std::map<std::string, raisim::VisualObject>& objList, const std::vector<double>& GRFs, const std::vector<double>& pf, const std::vector<double>& contact) {
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
        objList[objKey].offset = {pf[3 * i], pf[3 * i + 1], pf[3 * i + 2]};
        objList[objKey].scale = {0.2, 0.2, 0.005 * norm};  // Scaling based on the norm of the GRF vector

        // Set the rotation matrix
        objList[objKey].rotationOffset = rot;
    }
}

// void updatePolyline(raisim::VisualObject* visual, const casadi::DM& points) {
//     // Clear the old points from the polyline
//     visual->clearPoints();

//     // Add new points to the polyline
//     for (const auto& point : points) {
//         visual->addPoint(point);
//     }
// }
void ArmPDControl(std::vector<double> &tau_arm, double arm_pos[4], double arm_vel[4]){
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


void controller0(std::vector<raisim::ArticulatedSystem *> A1, LocoWrapper *loco_obj, NMPC_A1_KINOVA* nmpc_obj, size_t controlTick, bool* runMPC, Eigen::MatrixXd& MainData) {
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// INITIALIZE
    /////////////////////////////////////////////////////////////////////
    size_t loco_kind = TROT;                        // Gait pattern to use
    size_t pose_kind = POSE_COMB;                   // Pose type to use (if loco_kind is set to POSE)
    size_t settling = 8*ctrlHz;                   // Settling down
    size_t duration = 4*ctrlHz;                   // Stand up
    size_t loco_start = settling + duration;        // Start the locomotion pattern
    size_t stand_settle = 1*ctrlHz; // Settle down after stand up
    size_t trot_start = settling + duration + stand_settle; //This is where we can play starting trot
    // std::string basePath = "./tmp/";

    double *tau;
    std::vector<double> tau_arm;
    Eigen::VectorXd jointTorqueFF = Eigen::MatrixXd::Zero(TOTAL_DOF_A1_KINOVA+6,1);
    Eigen::VectorXd jointTorqueA1 = Eigen::MatrixXd::Zero(TOTAL_DOF_A1,1);
    Eigen::Vector4d jointTorqueKinova = Eigen::MatrixXd::Zero(TOTAL_DOF_KINOVA,1);
    Eigen::VectorXd jointPosTotal = Eigen::MatrixXd::Zero(TOTAL_DOF_A1_KINOVA+1,1); // +1 is for 4th Component of Quaternion 
    Eigen::VectorXd jointVelTotal = Eigen::MatrixXd::Zero(TOTAL_DOF_A1_KINOVA,1);
    static int conIndDes[4] = {1};
    // std::cout<<"RunMPC: "<<*runMPC<<std::endl;
    
    /////////////////////////////////////////////////////////////////////
    //////////////////////////// UPDATE STATE
    /////////////////////////////////////////////////////////////////////

    A1.back()->getState(jointPosTotal, jointVelTotal);
    // writeMatrixToFile(jointPosTotal, basePath+"jointPosTotal.txt");
    // writeMatrixToFile(jointVelTotal, basePath+"jointVelTotal.txt");
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
    double jpos_arm[4], jvel_arm[4];
    Eigen::Matrix<double, 3, 1> eul;
    Eigen::Matrix<double, 4, 1> quat;
    
    quat = jointPosTotal.block(3,0,4,1);
    quat_to_XYZ(quat,eul); // Convert quaternion to Euler angles
    // quat_to_ZYX(quat,eul_zyx); // Convert quaternion to Euler angles
    for(size_t i=0; i<3; ++i){
        jpos[i] = jointPosTotal(i);
        jvel[i] = jointVelTotal(i);
        jpos[i+3] = eul(i);
        jvel[i+3] = jointVelTotal(i+3);
    }
    for(size_t i=6; i<18; ++i){
        jpos[i] = jointPosTotal(i+1);
        jvel[i] = jointVelTotal(i);
    }
    for (size_t i = 0; i < 4; ++i) {
        jpos_arm[i] = jointPosTotal(i + 18 + 1);
        jvel_arm[i] = jointVelTotal(i + 18);
    }
    
//Setting Retrieving contact positions and forces from Randy. con is an object of a Randy's struct - it has all the info about footholds
    int force[4] = {0};
    for(auto &con: A1.back()->getContacts()){
        int conInd = con.getlocalBodyIndex();
        force[conInd/3-1] = 0;
        // force[conInd/3-1] = con.getNormal().e().norm();
    }
    
// Update data in the shared struct for LL controller
    memcpy(LLData0.q,jpos,18*sizeof(double));
    memcpy(LLData0.dq,jvel,18*sizeof(double));

    const int* contactMat = loco_obj->getConDes();
    memcpy(LLData0.ind,contactMat,4*sizeof(int));
    memcpy(conIndDes,contactMat,4*sizeof(int));

    // kinEst0(force,conIndDes,jpos,jvel,rotE);

    /////////////////////////////////////////////////////////////////////
    //////////////////////////// CONTROL
    /////////////////////////////////////////////////////////////////////
    // Update the desired torques
    if(controlTick < settling){ // Settle down
        // loco_obj->posSetup(jointPosTotal.head(3));
        double temp[18] = {0};
        tau = temp;
        // tau_arm = {0.0354537, 1.52263, -1.46063, 0};//intialize arm torques to zero
        // tau_arm = {0, 9, -4, 0};//intialize arm torques to zero
        
        tau_arm =  {0, 9.94638, -3.12554, 0};//intialize arm torques to zero
        ArmPDControl(tau_arm, jpos_arm, jvel_arm);
        loco_obj->initStandVars(jointPosTotal.block(0,0,3,1),(int)duration);
    }
    else 
    {
        if(controlTick >= settling & controlTick <= loco_start){ // Start standing
        loco_kind = STAND_MPC;
        }
        else if(controlTick > loco_start & controlTick <= trot_start){ // Start standing
        loco_kind = STAND_MPC;
        }
        else if(controlTick > trot_start){ // Start trot
        loco_kind = TROT;
        }

        //================================================================================================
        //========================================== HIGH LEVEL ==========================================
        //================================================================================================

        // TODO : RUN MPC AFTER EVERY 20ms (right now it is not running according to the control frequency)
        // TODO : Check how footholds are being set in LL controller, must be set by the MPC object

        updateData(GET_DATA, HL_DATA, &HLData0,0);
        // mpc_obj->updateState(HLData0.q, HLData0.dq, HLData0.ind, HLData0.toePos, HLData0.last_state1);
        nmpc_obj->updateState(HLData0.q, HLData0.dq, jpos_arm,  jvel_arm, HLData0.ind, HLData0.toePos, HLData0.last_state1, HLData0.domain, (controlTick+25));
        // writeMatrixToFile(HLData0.toePos, basePath+"toePos.txt");
        //Don't need last_state for single agent 
        if ( *runMPC == 1 && LLData0.domain < TOTALSTEPNUM)
        {
            // mpc_obj->run_NMPC();
            // HLData0.MPC_sol_ = mpc_obj->get_MPCsol();
            // HLData0.alpha_COM = mpc_obj->get_alphaCOM();
            // cout << "MPC_sol old: \n" << HLData0.MPC_sol_;

            *runMPC = 0;
            first_time0 = 0;

            HLData0.MPC_data_available = 1;
            // HLData0.domain = mpc_obj->getDomain();
            // HLData0.last_state0 = mpc_obj->get_lastState(); // for multi agent stuff only

            // NMPC portion
            // TO DO: State-feedback needs to be implemented 
            // For now NMPC is running on NMPC class contained state variables feedback
            // - [ ] Check if we can retrieve that no-feedback solution from NMPC and run low-level on it
            {

                nmpc_obj->run_NMPC();
                // std::cout << "\nBezier Coeffs:" << nmpc_obj->get_alphaCOM() << std::endl;
                // std::cout << "\nMPC Sol:" << nmpc_obj->get_MPCsol()   << std::endl;

                HLData0.MPC_data_available = 1;
                HLData0.alpha_COM = nmpc_obj->get_alphaCOM();
                HLData0.alpha_GRF = nmpc_obj->get_alphaGRF();
                // writeMatrixToFile(HLData0.alpha_COM, basePath+"HLData0_alpha_COM.txt");
                // writeMatrixToFile(HLData0.alpha_GRF, basePath+"HLData0_alpha_GRF.txt");
                HLData0.MPC_sol_ = nmpc_obj->get_MPCsol();
                HLData0.MPC_sol_GRF_ = nmpc_obj->get_MPCsolGRF();
                // writeMatrixToFile(HLData0.MPC_sol_, basePath+"HLData0_MPC_sol_.txt");
                // writeMatrixToFile(HLData0.MPC_sol_GRF_, basePath+"HLData0_MPC_sol_GRF_.txt");
                
            }
        }
        tau_arm = nmpc_obj->get_arm_input_torques();
        
        updateData(SET_DATA, HL_DATA, &HLData0,0);

        //================================================================================================
        //========================================== LOW LEVEL ===========================================
        //================================================================================================

        // TODO : Map MPC solution to full order torques, modify low-level controller

        updateData(GET_DATA, LL_DATA, &LLData0,0);
        // writeMatrixToFile(LLData0.alpha_COM, basePath+"LLData0_alpha_COM.txt");
        // writeMatrixToFile(LLData0.MPC_sol_, basePath+"LLData0_MPC_sol_.txt");
        loco_obj->set_MPC_DATA_W_GRF(LLData0.alpha_COM, LLData0.MPC_sol_, LLData0.alpha_GRF, LLData0.MPC_sol_GRF_,LLData0.MPC_data_available);
        loco_obj->calcZOHTauWithGRF(jpos,jvel,rotMatrixDouble,force,loco_kind,controlTick, runMPC);
        // loco_obj->calcZOHTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick, runMPC);
        // loco_obj->calcTau(jpos,jvel,rotMatrixDouble,force,STAND,controlTick, runMPC);
        tau = loco_obj->getTorque();

        LLData0.MPC_data_available = loco_obj->get_MPC_data_available();
        const int* contactMat = loco_obj->getConDes();
        Eigen::Matrix<double, 3, 4> toePosTmp = loco_obj->get_toePos();
        LLData0.toePos = toePosTmp;
        LLData0.domain = loco_obj->getDomain();
        LLData0.domain_change = loco_obj->hasDomainChanged();
        // LLData0.control_tick = controlTick;
        cout << "LLData0.domain: " << LLData0.domain << std::endl;
        memcpy(LLData0.ind, contactMat,4*sizeof(int));
        updateData(SET_DATA, LL_DATA, &LLData0,0);
    }
    
    // std::cout<<"tau_arm: "<<tau_arm<<std::endl;
    jointTorqueA1 = Eigen::Map< Eigen::Matrix<double,18,1> >(tau,18);
    jointTorqueKinova = Eigen::Map<Eigen::Matrix<double,4,1>>(tau_arm.data(), tau_arm.size());
    // jointTorqueKinova = Eigen::Matrix<double,4,1>::Zero();
    // writeMatrixToFile(jointTorqueA1, basePath+"jointTorqueA1.txt");
    // writeMatrixToFile(jointTorqueKinova, basePath+"jointTorqueKinova.txt");
    jointTorqueFF.block(0,0,18,1) = jointTorqueA1;
    jointTorqueFF.block(0,0,6,1).setZero(); //Raisim has torque with each DOF, so we need to set the first 6 (SRB COM) to zero
    // jointTorqueFF.block(0,0,18,1) = jointTorqueA1;
    // jointTorqueFF.block(0,0,18,1).setZero();
    jointTorqueFF.block(18,0,4,1) = jointTorqueKinova;
    //Gripper Torque
    Eigen::Matrix<double,6,1> gripperTorque = 0.1*Eigen::Matrix<double,6,1>::Ones();
    //Combine the torques for a1 kinmona and gripper
    jointTorqueFF.block(22,0,6,1) = gripperTorque;
    // std::cout << "jointTorqueFF: " << (jointTorqueKinova.transpose())(1) << std::endl;
    // jointTorqueFF.block(18,0,4,1) = Eigen::Matrix<double,4,1>{0,10,-4,0};
    // Set the desired torques
    A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    // A1.back()->setGeneralizedForce(jointTorqueFF.block(0,0,18,1));
    A1.back()->setGeneralizedForce(jointTorqueFF);
    MainData.block(controlTick, 0, 1, 22) = jointTorqueFF.transpose();
};

int main(int argc, char *argv[]) {


    NMPC_A1_KINOVA* nmpc;
    nmpc = new NMPC_A1_KINOVA(argc, argv);
    // nmpc->code_gen();
    nmpc->init();




    // nmpc->generateReferenceTrajectory();




    // size_t mpc_iter = 0;
    // while (mpc_iter < nmpc->params.sim_length/ nmpc->params.dt) {
    //     nmpc->run_NMPC();
    //     mpc_iter++;
    // }
    // nmpc->logData();

    // return 0;
    

    // ============================================================ //
    std::fstream outFile("/home/trec/Desktop/A1_RaiSim_Outputs/failingDistances.txt", std::ios::out);

    const int NUMBER_OF_SIMS = 1;

    const float threshold = 0.4;

    bool shared_data_backed_up = 0;
    // ============================================================ //
    // =================== SETUP RAISIM/VISUALS =================== //
    // ============================================================ //
    /// create raisim world

    raisim::World::setActivationKey(raisim::loadResource("activation.raisim"));
    raisim::World world;
    world.setTimeStep(simfreq_raisim);

    raisim::OgreVis *vis = raisim::OgreVis::get();

    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(1792, 1200); // Should be evenly divisible by 16!!
    vis->setImguiSetupCallback(imguiSetupCallback); // These 2 lines make the interactable gui visible
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setKeyboardCallback(raisimKeyboardCallback);
    vis->setSetUpCallback(setupCallback);
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
    // auto* visual = vis->createGraphicalObject("curve", raisim::OgreVis::Visuals::PolyLine);  
    // auto* visual = raisim::OgreVis::get()->createGraphicalObject("curve", raisim::OgreVis::Visuals::PolyLine);



    auto& list = vis->getVisualObjectList();

    float box_width = 0.25;
    float box_z = 0.15;
    auto boxd = world.addBox(box_width,box_width,0.3,0.1);
    vis->createGraphicalObject(boxd, "boxd", "green");

    // Mimic the rough terrain

    std::vector<std::string> colors = {"red", "green", "blue", "yellow", "orange", "purple", "pink", "brown", "grey", "white"};
    std::random_device rd1, rd2, rd3;  
    std::mt19937 gen1(rd1()), gen2(rd2()), gen3(rd3());
    std::uniform_real_distribution<> dis_height(0.0, 0.02); // Random heights between 0 and 1 cm
    std::uniform_real_distribution<> dis_width(0.03, 0.2); // Random width between 3 and 7 cm
    std::uniform_real_distribution<> dis_length(1, 5); // Random length between 3 and 7 cm
    std::uniform_int_distribution<> dis_color(0, colors.size() - 1); // Distribution for color indices

    int num_blocks = 48; // Number of blocks
    std::vector<raisim::Box*> wood_blocks;

    int num_rows = num_blocks/1; // Number of rows for block placement
    float spacing_factor = 1.5; // Adjust spacing between blocks

    if(rough_terrain_en) {
        for (size_t i = 0; i < num_blocks; i++) {
            float wood_height = dis_height(gen1); // Generate a random height for each box
            float wood_width = dis_width(gen2);   // Generate a random width for each box
            float wood_length = dis_length(gen3); // Generate a random length for each box
            int color_index = dis_color(gen1);    // Generate a random color index

            wood_blocks.push_back(world.addBox(wood_width, wood_length, wood_height, 0.1)); // Add the box to the world with random dimensions
            vis->createGraphicalObject(wood_blocks.back(), "wood_block" + std::to_string(i), colors[color_index]);

            // Set the position of the box in a grid pattern
            int row = i / num_rows;
            int col = i % num_rows;
            wood_blocks.back()->setPosition(col * wood_width * spacing_factor, row * wood_length * spacing_factor, wood_height / 2);
        }
    };


    // Populate the obstacles 
    // int number_of_obs = NUMBER_OF_OBS;

    std::vector<raisim::Box*> obstacles;
    if (obstacle_en)
    {
        for (size_t i = 0; i < NUMBER_OF_OBS; i++)
        {
            obstacles.push_back(world.addBox(box_width,box_width,0.3,0.1));
            vis->createGraphicalObject(obstacles.back(), "box"+std::to_string(i), "red");
        }
    }
    std::vector<raisim::Box*> pushboxes;
    if (push_box_en)
    {   
        float push_box_width = 0.25;
        float push_box_length = 1.0;
        float push_box_height = 0.75;
        float push_box_mass = 2.0;
        float push_box_x = 2.0;
        float push_box_y = 0.0;
        pushboxes.push_back(world.addBox(push_box_length,push_box_width,push_box_height,push_box_mass));
        vis->createGraphicalObject(pushboxes.back(), "push_box", "red");
        pushboxes.back()->setPosition(push_box_x,push_box_y,push_box_height/2);
    }

    // ============================================================ //
    // ======================= SETUP Robot ======================== //
    // ============================================================ //
    std::vector<raisim::ArticulatedSystem*> A1;
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new_w_kinova.urdf")));   // with NMPC and LL
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova.urdf")));   // with NMPC and LL  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_collision.urdf")));   // with NMPC and LL  
    A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_fingers.urdf")));   // with NMPC and LL  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_w_kinova_light.urdf")));   // with NMPC and LL with light URDF  
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_trunk.urdf")));             // For NMPC only
    // A1.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified_new.urdf"))); 
    
    vis->createGraphicalObject(A1.back(), "A1");
    A1.back()->setName("A1_Robot");

    if(payload_en)
    {
        float payload_width = 0.1;
        float payload_length = 0.1;
        float payload_height = 0.1;
        float payload_mass = 2.0;
        float payload_x = 0.6;
        float payload_y = 0.0;
        float payload_z = 0.15;
        raisim::Box* payload = world.addBox(payload_length,payload_width,payload_height,payload_mass);
        vis->createGraphicalObject(payload, "payload", "blue");
        payload->setPosition(payload_x,payload_y,payload_z);
        //Add stifflength constraint to the arm end effector and payload
        size_t ee_index = A1.back()->getBodyIdx("j2n4s300_link_4");
        auto wire1 = world.addStiffWire(A1.back(),ee_index , {0,0,0}, payload, 0, {0,0,payload_height/2}, 0.01);
        //visualize the wire
        wire1->setVisualizationWidth(0.01);
        vis->createGraphicalObject(wire1, "wire1", "black");  
        std::cout << "Width : " << wire1->getVisualizationWidth()<< std::endl; 

    }

    if(gripper_grasp_en)
    {
        //Place object inside the gripper
        float object_width = 0.05;
        float object_length = 0.05;
        float object_height = 0.05;
        float object_mass = 0.5;
        float object_x = 0.58;
        float object_y = 0.0;
        float object_z = 0.27;
        raisim::Box* object = world.addBox(object_length,object_width,object_height,object_mass);
        vis->createGraphicalObject(object, "object", "blue");
        object->setPosition(object_x,object_y,object_z);
        //load cup urdf file
        /* raisim::ArticulatedSystem* cup = world.addArticulatedSystem(raisim::loadResource("A1/cup.urdf"));
        vis->createGraphicalObject(cup, "cup");
        cup->setName("cup");
        //Add stifflength constraint to the arm end effector and payload
        float cup_x = 0.6;
        float cup_y = 0.0;
        float cup_z = 0.45;
        cup->setGeneralizedCoordinate({cup_x,cup_y,cup_z,1,0,0,0}); */
        
                

        
    }
    
    std::vector<raisim::Box*> payloads;
    if (payload_en)
    {
        payloads.push_back(world.addBox(0.1,0.1,0.1,0.1));
        vis->createGraphicalObject(payloads.back(), "payload", "blue");
        //attach payload to the end effector
        // Get the body index of the Kinova arm's end effector
        auto endEffectorIdx = A1.back()->getFrameIdxByName("end_effector_frame");
        auto constraint = world.addStiffWire(
        A1.back(), endEffectorIdx, {0.1, 0.1, 0.1},
        payloads.back(), 0, {0, 0, 0}, 0.1);




    }
    


    // ============================================================ //
    // ==================== SETUP MORE Robots ===================== //
    // ============================================================ //

    // AGENT 2
    // std::vector<raisim::ArticulatedSystem*> A2;
    // A2.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
    // vis->createGraphicalObject(A2.back(), "A2");
    // A2.back()->setName("A1_Robot_2");

    for (size_t sim = 0; sim < NUMBER_OF_SIMS; ++sim)
    {
        boxd->setPosition(GOAL_X,GOAL_Y+100,box_z);
        Eigen::Matrix<double, 2*NUMBER_OF_AGENTS, 1> Pstart;
        // Define the matrices
        Eigen::Matrix<double, 2, NUMBER_OF_OBS> Pobs;
        Eigen::Matrix<double, 2, NUMBER_OF_OBS> Pobs_real;


     /*    std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        
        // Uniform distributions for the required ranges
        std::uniform_real_distribution<> dis_x(0, 9.0); // For x values 
        std::uniform_real_distribution<> dis_y(-2.0, 2.0); // For y values 
        std::uniform_real_distribution<> dis_uncertainty(-0.6, 0.6); // For uncertainties

        
        srand(static_cast<unsigned int>(time(nullptr)));

        // Pobs   <<       2.2,            1,         1,             1,              1,                3,             3,        3,       -100,
        //         0.9,            1,     -0.75,             2,          -1.75,              0.5,         -0.25,    -1.75,    -0.5 + 100; 


        for (int col = 0; col < Pobs.cols(); ++col) {
            // Generate random values in the specified ranges
            double randX = dis_x(gen);
            double randY = dis_y(gen);

            // Assign these values to Pobs
            Pobs(0, col) = randX;
            Pobs(1, col) = randY;

            if(!obstacle_en)
            {
                // Add 100 in y values to move obstacles out of the way 
                Pobs.row(1).array() += 100;
            }

            // Generate uncertainty
            double uncertaintyX = dis_uncertainty(gen);
            double uncertaintyY = dis_uncertainty(gen);

            // Add uncertainty to create Pobs_real
            Pobs_real(0, col) = Pobs(0, col);// + uncertaintyX;
            Pobs_real(1, col) = Pobs(1, col);// + uncertaintyY;

            // Ensure Pobs_real stays within bounds
            Pobs_real(0, col) = std::min(std::max(Pobs_real(0, col), 1.0), 9.0);
            Pobs_real(1, col) = std::min(std::max(Pobs_real(1, col), -3.0), 3.0);
        } */
    
        for (size_t solver = 0; solver < 1; ++solver)
        {
            bool checkifrunMPC0 = 1;
            // bool checkifrunMPC1 = 0;
            // bool checkifrunMPC2 = 0;
            // bool checkifrunMPC3 = 0;
            first_time0 = 1;
            // first_time1 = 1;
            // first_time2 = 1;
            // first_time3 = 
            sec_time = 1;

            // AGENT 2
            // HLData0 = HLData0_Backup;
            // updateData0(SET_DATA, HL_DATA, &HLData0);
            // HLData1 = HLData1_Backup;
            // updateData1(SET_DATA, HL_DATA, &HLData1);
            // LLData0 = LLData0_Backup;
            // updateData0(SET_DATA, LL_DATA, &LLData0);
            // LLData1 = LLData1_Backup;
            // updateData1(SET_DATA, LL_DATA, &LLData1);

            // ============================================================ //
            // =================== SETUP ENVIRONMENT ====================== //
            // ============================================================ //
            

            /* std::cout << "Pobs:\n" << Pobs << "\n\n";
            std::cout << "Pobs_real (with uncertainty):\n" << Pobs_real << "\n";

            // Pstart << 0.0, 0.0, 0.0, -0.9, -1, 0, -1, -0.9;
            Pstart << 0.0, 0.0;
        
            // Populate the obstacles 
            int obs_iter = 0; */

            /* if (obstacle_en)
            {
                for (auto it = obstacles.begin(); it != obstacles.end(); ++it) {
                    raisim::Box* obstacle = *it;
                    // Use 'obstacle' as needed
                    // obstacle->setPosition(Pobs_real(0,obs_iter),Pobs_real(1,obs_iter),box_z);
                    obstacle->setPosition(Pobs(0,obs_iter),Pobs(1,obs_iter),box_z);
                    obstacle->setMass(0.2);
                    obstacle->setOrientation(0,0,0,1);
                    obs_iter++;
                }
            } */

            // SET AGENT POSITIONS FOR THE NEW SIM
            // TODO 
            // {Pstart(0), Pstart(1), 0.12, 1, 0, 0, 0,0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6}
            // A1.back()->setGeneralizedCoordinate({Pstart(0), Pstart(1), 0.12, 1, 0, 0, 0});

            // A1.back()->setGeneralizedCoordinate({0, 0, 0.8, 1, 0, 0, 0});       // For NMPC only
            A1.back()->setGeneralizedCoordinate({0.0, 0.0, 0.12, 1, 0, 0, 0,0.0, Pi/3, -2.8, 0.0, Pi/3, -2.8, 0.0, Pi/3, -2.8, 0.0, Pi/3, -2.8, 0.0,3.49066,5.67232,0.0, 0.75,0.75,0.75,0.75,0.75,0.75 });
            // A1.back()->setGeneralizedCoordinate({0.0, 0.0, 0.12, 1, 0, 0, 0,0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            A1.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            LocoWrapper* loco_obj = new LocoWrapper(argc,argv);
            loco_obj->setAgentID(0); //Agent ID = 0
            loco_obj->setPstart(Pstart);
            loco_obj->setPobs(Pobs);
            // loco_obj->generateReferenceTrajectory();
            
            // AGENT 2
            // A2.back()->setGeneralizedCoordinate({Pstart(2), Pstart(3), 0.12, 1, 0, 0, 0,
            //                                     0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            // A2.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

            // LocoWrapper* loco_obj2 = new LocoWrapper(argc,argv);
            // loco_obj2->setAgentID(1); //Agent ID = 1
            // loco_obj2->setPstart(Pstart);
            // loco_obj2->setPobs(Pobs);
            // loco_obj2->generateReferenceTrajectory();

            // // Third Robot 
            // std::vector<raisim::ArticulatedSystem*> A3;
            // A3.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
            // vis->createGraphicalObject(A3.back(), "A3");
            // A3.back()->setGeneralizedCoordinate({Pstart(4), Pstart(5), 0.12, 1, 0, 0, 0,
            //                                     0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            // A2.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            // A2.back()->setName("A1_Robot_3");
            // LocoWrapper* loco_obj3 = new LocoWrapper(argc,argv);
            // loco_obj3->setAgentID(2); //Agent ID = 1
            // loco_obj3->setPstart(Pstart);
            // loco_obj3->setPobs(Pobs);
            // loco_obj3->generateReferenceTrajectory();

            // // Fourth Robot 
            // std::vector<raisim::ArticulatedSystem*> A4;
            // A4.push_back(world.addArticulatedSystem(raisim::loadResource("A1/A1_modified.urdf")));
            // vis->createGraphicalObject(A4.back(), "A4");
            // A4.back()->setGeneralizedCoordinate({Pstart(6), Pstart(7), 0.12, 1, 0, 0, 0,
            //                                     0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6, 0.0, Pi/3, -2.6});
            // A4.back()->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
            // A4.back()->setName("A1_Robot_4");
            // LocoWrapper* loco_obj4 = new LocoWrapper(argc,argv);
            // loco_obj4->setAgentID(3); //Agent ID = 1
            // loco_obj4->setPstart(Pstart);
            // loco_obj4->setPobs(Pobs);
            // loco_obj4->generateReferenceTrajectory();

            // ============================================================ //
            // ================= VIEW AND RECORDING OPTIONS =============== //
            // ============================================================ //
            raisim::gui::showContacts = false;
            raisim::gui::showForces = false;
            raisim::gui::showCollision = false;
            raisim::gui::showBodies = true;

            std::string cameraview = "iso"; // iso, isoside, side, front, top
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
            // std::string filename = "inplace_sim";

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
            long simcounter = 0;
            static bool added = false;

            // ============================ MPC =========================== //
            // MPC_dist* MPC_Agent0;
            // MPC_dist* MPC_Agent1;
            // MPC_dist* MPC_Agent2;
            // MPC_dist* MPC_Agent3;

            // std::cout << "Pstart: \n" << Pstart << "\n";
            // std::cout << "Pobs: \n" << Pobs << "\n";
            // std::cout << "Pobs_real: \n" << Pobs_real << "\n";

            // std::cin.get();

            /* MPC_Agent0 = new MPC_dist();
            MPC_Agent0->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
            MPC_Agent0->setPstart(Pstart);
            MPC_Agent0->setPobs(Pobs);
            MPC_Agent0->setPobs_real(Pobs_real); */

            nmpc->setAgentID(0);    //MPC_Agent1 = new MPC_dist();
            nmpc->setPstart(Pstart);
            nmpc->setPobs(Pobs);
            nmpc->setPobs_real(Pobs);
            // nmpc->generateReferenceTrajectory();
            // cout << "Can we reach here SIM 1"; cin.get();
            // MPC_Agent0->generateReferenceTrajectory();
            // cout << "Can we reach here SIM 2"; cin.get();

            // AGENT 2
            // MPC_Agent1 = new MPC_dist();
            // MPC_Agent1->setAgentID(1);    //MPC_Agent1 = new MPC_dist();
            // MPC_Agent1->setPstart(Pstart);
            // MPC_Agent1->setPobs(Pobs);
            // MPC_Agent1->setPobs_real(Pobs_real);
            // MPC_Agent1->generateReferenceTrajectory();


            // MPC_Agent2 = new MPC_dist();
            // MPC_Agent2->setAgentID(2);    //MPC_Agent1 = new MPC_dist();
            // MPC_Agent2->setPstart(Pstart);
            // MPC_Agent2->setPobs(Pobs);
            // MPC_Agent2->setPobs_real(Pobs_real);
            // MPC_Agent2->generateReferenceTrajectory();

            // MPC_Agent3 = new MPC_dist();
            // MPC_Agent3->setAgentID(3);    //MPC_Agent1 = new MPC_dist();
            // MPC_Agent3->setPstart(Pstart);
            // MPC_Agent3->setPobs(Pobs);
            // MPC_Agent3->setPobs_real(Pobs_real);
            // MPC_Agent3->generateReferenceTrajectory();
            double failingDistance0 = 10;
            double failingDistance1 = 10;

            // Eigen::MatrixXd AppliedTorque(22,simlength);
            Eigen::MatrixXd MainData;
            MainData.resize(simlength,22);


            while (!vis->getRoot()->endRenderingQueued() && simcounter < simlength-1){

                // nmpc->run_NMPC();
                // std::vector<double> com_pos = nmpc->get_com_pos();
                // A1.back()->setGeneralizedCoordinate({com_pos[0], com_pos[1], com_pos[2], 1, 0, 0, 0});

                // cout << "Sim: " << sim << "\n";
                size_t dist_start = 3*ctrlHz;               // Start the disturbance (if any)
                size_t dist_stop  = dist_start+8000;             // Stop the disturbance (if any)
                std::string axis = "lat";// "bkwdlat";                          // Axis of the disturbance options - 
                double dist_mag = 25.0;                      // Magnitude of the disturbance
                if (disturbance_en)
                { 
                    disturbance(A1, list, dist_start, dist_stop, simcounter,dist_mag, axis);
                }


                controller0(A1,loco_obj, nmpc, simcounter, &checkifrunMPC0, MainData);
                std::vector<double> GRF = nmpc->get_GRF();
                std::vector<double> feet_vec = nmpc->get_Pf();
                std::vector<double> contacts = nmpc->get_contacts();
                plotGRFs(list, GRF, feet_vec, contacts);
                // controller1(A2,loco_obj2, MPC_Agent1, simcounter, &checkifrunMPC1);
                // MPC_Agent0->updateDistance_to_fail();
                // MPC_Agent1->updateDistance_to_fail();

                // failingDistance0 = MPC_Agent0->getDistance_to_fail();
                // failingDistance1 = MPC_Agent1->getDistance_to_fail();
                // controller2(A3,loco_obj3, MPC_Agent2, simcounter, &checkifrunMPC2);
                // controller3(A4,loco_obj4, MPC_Agent3, simcounter, &checkifrunMPC3);
                // controller(A3,loco_obj3,simcounter);
                // controller(A4,loco_obj4,simcounter);
                

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

            nmpc->logData();
    
            writeMatrixToFile(MainData, "./tmp/MainData.txt");

            // outFile << failingDistance0 << "," << failingDistance1 << std::endl;

            delete loco_obj;
            // delete loco_obj2;
            // delete MPC_Agent0;
            // delete MPC_Agent1;
        }
    }
    outFile.close();

    // End recording if still recording
    if (vis->isRecording())
        vis->stopRecordingVideoAndSave();
    
    /// terminate the app
    vis->closeApp();
    return 0;
}
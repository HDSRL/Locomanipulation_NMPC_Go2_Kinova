%% Testing only

%% Load raisim LL data
filename = {[ '/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/tmp/LL_agent_0.txt']};
LL_data = parseData(filename);

%% LL plots
%plot phase
f1 = figure('Name','Data Logged from Low Level')
subplot(3,3,1)
plot(LL_data.time,LL_data.hd(:,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Position (m)')
legend('X','Y','Z');
title('LL Data hd - Position')
grid on;


subplot(3,3,2)
plot(LL_data.time,LL_data.hd(:,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Angle (rad)')
legend('R','P','Y');
title('LL Data hd - roll pitch yaw')
grid on;

subplot(3,3,3)
plot(LL_data.time,LL_data.dhd(:,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Velocity (m/s)')
legend('X','Y','Z');
title('LL Data dhd - Velocity')
grid on;

subplot(3,3,4)
plot(LL_data.time,LL_data.dhd(:,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Angle (rad/s)')
legend('R','P','Y');
title('LL Data dhd - Omega')
grid on;

subplot(3,3,5)
plot(LL_data.time,LL_data.pos(:,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Position')
legend('X','Y','Z');
title('LL data Position')
grid on;


subplot(3,3,6)
plot(LL_data.time,LL_data.outputs(:,4:6)+LL_data.hd(:,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Angle (rad)')
legend('R','P','Y');
title('LL data RPY')
grid on;


subplot(3,3,7)
plot(LL_data.time,LL_data.vel(:,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Velocity (m/s)')
legend('X','Y','Z');
title('LL data Velocity')
grid on;

subplot(3,3,8)
plot(LL_data.time,LL_data.vel(:,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Angle (rad/s)')
legend('R','P','Y');
title('LL data Angular Velocity')
grid on;


subplot(3,3,9)
plot(LL_data.time,LL_data.hd(:,1:3),'--','LineWidth',1.5);
hold on
plot(LL_data.time,LL_data.pos(:,1:3),':','LineWidth',1.5);
xlabel('time(s)');
ylabel('Position (m)')
legend('X_{commanded}','Y_{commanded}','Z_{commanded}','X_{achieved}','Y_{achieved}','Z_{achieved}');
title('Comparison LL Position')
grid on;

% subplot(3,3,9)
% plot(ll_data.time,ll_data.fdes-ll_data.qpforce,'linewidth',1.5);
% xlabel('time(s)');
% ylabel('grf error (fdes - f)')
% legend('x_{fr}','y_{fr}','z_{fr}',...
%     'x_{fl}','y_{fl}','z_{fl}',...
%     'x_{rr}','y_{rr}','z_{rr}',...
%     'x_{rl}','y_{rl}','z_{rl}');
% title('ll data grf (f desired - f achieved)')
% grid on;
%%

figure('Name','LL:Output PLot')
subplot(2,2,1)
plot(LL_data.time,LL_data.outputs(:,1:3),':','LineWidth',1.5);
xlabel('time(s)');
ylabel('Position (m)')
legend('X_{error}','Y_{error}','Z_{error}');
title('Outputs LL Position')
grid on;

subplot(2,2,2)
plot(LL_data.time,LL_data.outputs(:,4:6),':','LineWidth',1.5);
xlabel('time(s)');
ylabel('Orientation (rad)')
legend('R_{error}','P_{error}','Y_{error}');
title('Outputs LL Orientation')
grid on;

subplot(2,2,3)
plot(LL_data.time,LL_data.outputs(:,7:9),':','LineWidth',1.5);
xlabel('time(s)');
ylabel('Velocity (m/s)')
legend('X_{error}','Y_{error}','Z_{error}');
title('Outputs LL Velocity')
grid on;

subplot(2,2,4)
plot(LL_data.time,LL_data.outputs(:,10:12),':','LineWidth',1.5);
xlabel('time(s)');
ylabel('Omega(rad/s)')
legend('R_{error}','P_{error}','Y_{error}');
title('Outputs LL Orientation rate')
grid on;





%% Testing GRF tracking
f2 = figure('Name',' GRF: LL Data Desired minus achieved QP force')
subplot(2,2,1)
plot(LL_data.time,LL_data.fDes(:,1:3)-LL_data.QPforce(1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f) (N)')
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF: Difference between desired and actual (fdes -f) FR')
grid on;

subplot(2,2,2)
plot(LL_data.time,LL_data.fDes(:,4:6)-LL_data.QPforce(4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f) (N)')
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF: Difference between desired and actual(fdes -f) FL')
grid on;

subplot(2,2,3)
plot(LL_data.time,LL_data.fDes(:,7:9)-LL_data.QPforce(7:9),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f) (N)')
legend('X_{RR}','Y_{RR}','Z_{RR}');
title('GRF: Difference between desired and actual (fdes -f) RR')
grid on;

subplot(2,2,4)
plot(LL_data.time,LL_data.fDes(:,10:12)-LL_data.QPforce(10:12),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f) (N)')
legend('X_{RL}','Y_{RL}','Z_{RL}')
title('GRF: Difference between desired and actual (fdes -f) RR')
grid on;


f3 = figure('Name','Outputs and dOutputs XYZ of LL')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.outputs(1:end-1000,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Outputs y (m)')
title('XYZ Position Output of LL')
legend('X','Y','Z')
grid on;

subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.doutputs(1:end-1000,1:3));
xlabel('time(s)');
ylabel('dOutputs y (m/s)')
legend('X','Y','Z')
title('XYZ Velocity Output of LL')
grid on;



f4 = figure('Name','Outputs and dOutputs RPY of LL')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.outputs(1:end-1000,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Outputs y (rad)')
title('RPY Orientation Output of LL')
legend('R','P','Y')
grid on



subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.doutputs(1:end-1000,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('dOutputs y (rad/s)')
legend('R','P','Y')
title('RPY Angular Velocity Output of LL')
grid on;


f5 = figure('Name','Commanded XYZ Position/Velocity from NMPC to LL (hd/dhd)')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.hd(1:end-1000,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('hd (m)')
title('Commanded XYZ Position from NMPC to LL')
legend('X','Y','Z')
grid on;

subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.dhd(1:end-1000,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('dhd (m/s)')
title('Commanded XYZ Velocity from NMPC to LL')
legend('X','Y','Z')
grid on;





f6 = figure('Name','Commanded RPY Orientation/Angular Velocity from NMPC to LL (hd/dhd)')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.hd(1:end-1000,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('hd (rad)')
title('Commanded RPY Orientation from NMPC to LL')
legend('R','P','Y')
grid on;

subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.dhd(1:end-1000,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('dhd')
title('Commanded RPY Angular Velocity from NMPC to LL')
legend('R','P','Y')
grid on;


f7 = figure('Name','GRF: Desired from NMPC to LL (fdes)')
subplot(2,2,1)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF fdes (N)');
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF Desired from NMPC to LL (FR)')
grid on;

subplot(2,2,2)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF fdes (N)');
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF Desired from NMPC to LL (FL)')
grid on;


subplot(2,2,3)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,7:9),'LineWidth',1.5);
xlabel('time(s)');
ylabel('GRF fdes (N)');
legend('X_{RR}','Y_{RR}','Z_{RR}');
title('GRF Desired from NMPC to LL (RR)')
grid on;

subplot(2,2,4)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,10:12));
xlabel('time(s)');
ylabel('GRF fdes (N)');
legend('X_{RL}','Y_{RL}','Z_{RL}');
title('GRF Desired from NMPC to LL (RL)')
grid on;




f8 = figure('Name','GRF: Output from (QP force)')
subplot(2,2,1)
plot(LL_data.time(1:end-1000),LL_data.QPforce(1:end-1000,1:3));
xlabel('time(s)');
ylabel('GRF QP force (N)');
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF Output from LL (FR)')
ylim([-100,250])
grid on;

subplot(2,2,2)
plot(LL_data.time(1:end-1000),LL_data.QPforce(1:end-1000,4:6));
xlabel('time(s)');
ylabel('GRF QP force (N)');
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF Output from LL (FL)')
ylim([-100,250])
grid on;

subplot(2,2,3)
plot(LL_data.time(1:end-1000),LL_data.QPforce(1:end-1000,7:9));
xlabel('time(s)');
ylabel('GRF QP force (N)');
legend('X_{RR}','Y_{RR}','Z_{RR}');
title('GRF Output from LL (RR)')
ylim([-100,250])
grid on;


subplot(2,2,4)
plot(LL_data.time(1:end-1000),LL_data.QPforce(1:end-1000,10:12));
xlabel('time(s)');
ylabel('GRF QP force (N)');
legend('X_{RL}','Y_{RL}','Z_{RL}');
title('GRF Output from LL (RL)')
ylim([-100,250])
grid on;



colors_des = ["#0072BD","#FF0000","#EDB120"];
colors_achieved = ["#00FFFF","#D95319","#FF8800"];
f9 = figure('Name','GRF: Comparison of NMPC commanded and achieved')
for i =1:3
subplot(2,2,1)
plot(LL_data.time,LL_data.QPforce(:,i),'-','LineWidth',1.5,'Color',colors_achieved(i));
hold on
plot(LL_data.time,LL_data.fDes(:,i),'--','LineWidth',1.5,'Color',colors_des(i));
end
xlabel('time(s)');
ylabel('GRF fdes vs achieved (N)');
legend('X_{FR achieved}','X_{FR commanded}',...
    'Y_{FR achieved}','Y_{FR commanded}',...
    'Z_{FR achieved}', 'Z_{FR commanded}')
title('GRF Comparison of NMPC commanded and achieved (FR)')
ylim([-100,200])
grid on;

subplot(2,2,2)
for i =1:3
subplot(2,2,2)
plot(LL_data.time,LL_data.QPforce(:,i+3),'-','LineWidth',1.5,'Color',colors_achieved(i));
hold on
plot(LL_data.time,LL_data.fDes(:,i+3),'--','LineWidth',1.5,'Color',colors_des(i));
end
xlabel('time(s)');
ylabel('GRF fdes vs achieved (N)');
legend('X_{FL achieved}','X_{FL commanded}',...
    'Y_{FL achieved}','Y_{FL commanded}',...
    'Z_{FL achieved}', 'Z_{FL commanded}')
title('GRF Comparison of NMPC commanded and achieved (FL)')
ylim([-100,200])
grid on;

subplot(2,2,3)
for i =1:3
plot(LL_data.time,LL_data.QPforce(:,i+6),'-','LineWidth',1.5,'Color',colors_achieved(i));
hold on
plot(LL_data.time,LL_data.fDes(:,i+6),'--','LineWidth',1.5,'Color',colors_des(i));
end
xlabel('time(s)');
ylabel('GRF fdes vs achieved (N)');
legend('X_{RR achieved}','X_{RR commanded}',...
    'Y_{RR achieved}','Y_{RR commanded}',...
    'Z_{RR achieved}', 'Z_{RR commanded}')
title('GRF Comparison of NMPC commanded and achieved (RR)')
ylim([-100,200])
grid on;

subplot(2,2,4)
for i =1:3
plot(LL_data.time,LL_data.QPforce(:,i+9),'-','LineWidth',1.5,'Color',colors_achieved(i));
hold on
plot(LL_data.time,LL_data.fDes(:,i+9),'--','LineWidth',1.5,'Color',colors_des(i));
end
xlabel('time(s)');
ylabel('GRF fdes vs achieved (N)');
legend('X_{RL achieved}','X_{RL commanded}',...
    'Y_{RL achieved}','Y_{RL commanded}',...
    'Z_{RL achieved}', 'Z_{RL commanded}')
title('GRF Comparison of NMPC commanded and achieved RL')
ylim([-100,200])
grid on;

f20 = figure('Name','Motor Torques: Commanded by LL')
subplot(2,2,1)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,1:3),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{FR}','HipPitch_{FR}','Knee_{FR}');
title('Motor Torques from LL (FR)')
ylim([-30,30])
grid on;

subplot(2,2,2)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,4:6),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{FL}','HipPitch_{FL}','Knee_{FL}');
title('Motor Torques from LL (FL)')
ylim([-30,30])
grid on;

subplot(2,2,3)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,7:9),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{RR}','HipPitch_{RR}','Knee_{RR}');
title('Motor Torques from LL (RR)')
ylim([-30,30])
grid on;


subplot(2,2,4)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,10:12),'LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{RL}','HipPitch_{RL}','Knee_{RL}');
title('Motor Torques from LL (RL)')
ylim([-30,30])
grid on;







%% NMPC Plots
%plot phase
% filename = '/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/tmp/A1_KINOVA_NMPC_x_log';
filename = '/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/tmp/NMPC_x_log';
dd = [-0.1 0 0.1]';
NMPC_data = readmatrix(filename);
NMPC_time = NMPC_data(:,1);
NMPC_x_sol_1 = NMPC_data(:,2:13);
NMPC_qk_SRB = NMPC_data(:,14:25);
NMPC_ukSRB_ = NMPC_data(:,26:37);
NMPC_q_des_ = NMPC_data(:,38:49);


%% Plot data
f10 = figure('Name','Position/Velocity NMPC Solution XYZ vs NMPC Reference')
subplot(2,1,1)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Position(m)')
legend('X_{NMPC}','X_{ref}','Y_{NMPC}','Y_{ref}','Z_{NMPC}','Z_{ref}');
title('Position: NMPC XYZ Reference vs Solution')
ylim([-0.1,0.5])
grid on;

subplot(2,1,2)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i+3),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i+3),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Velocity(m/s)')
legend('X_{NMPC}','X_{ref}','Y_{NMPC}','Y_{ref}','Z_{NMPC}','Z_{ref}');
title('Velocity: NMPC Reference XYZ vs Solution')
ylim([-0.1,0.5])
grid on;


f11 = figure('Name','Orientation/AngularVelocity NMPC Solution RPY vs Reference')
subplot(2,1,1)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i+6),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i+6),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Angle(rad)')
legend('R_{sol}','R_{ref}','P_{sol}','P_{ref}','Y_{sol}','Y_{ref}')
title('Orientation: NMPC Solution RPY vs Reference')
grid on;
subplot(2,1,2)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i+9),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i+9),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Anglular velocity(rad/s)')
legend('R_{sol}','R_{ref}','P_{sol}','P_{ref}','Y_{sol}','Y_{ref}')
title('Angular Velocity: NMPC Solution RPY vs Reference')
grid on;

f12 = figure('Name','Position/Velocity NMPC Solution XYZ vs Feedback from LL')
subplot(2,1,1)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Position(m)')
legend('X_{sol}','X_{fbk}','Y_{sol}','Y_{fbk}','Z_{sol}','Z_{fbk}')
title('Position: NMPC Solution XYZ vs Feedback from LL')
grid on;

subplot(2,1,2)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i+3),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i+3),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Velocity (m/s)')
legend('X_{sol}','X_{fbk}','Y_{sol}','Y_{fbk}','Z_{sol}','Z_{fbk}')
title('Velocity: NMPC Solution XYZ vs Feedback from LL')
grid on;


f13 = figure('Name','Orientation/Angular Velocity NMPC Solution XYZ vs Feedback from LL')
subplot(2,1,1)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i+6),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i+6),'--','LineWidth',1.5,'Color',colors_des(i))
end

xlabel('Time(s)')
ylabel('Angle (rad)')
legend('R_{sol}','R_{fbk}','P_{sol}','P_{fbk}','Y_{sol}','Y_{fbk}')
title('Orientation: NMPC Solution XYZ vs Feedback from LL')
grid on;


subplot(2,1,2)
for i=1:3
    plot(NMPC_time,NMPC_x_sol_1(:,i+9),'-','LineWidth',1.5,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i+9),'--','LineWidth',1.5,'Color',colors_des(i))
end
xlabel('Time(s)')
ylabel('Angular Velocity (rad/s)')
legend('R_{sol}','R_{fbk}','P_{sol}','P_{fbk}','Y_{sol}','Y_{fbk}')
title('Angular Velocity: NMPC Solution XYZ vs Feedback from LL ')
grid on;



f14 = figure('Name','GRFs: NMPC Solution GRFs vs LL commanded GRFs')
subplot(2,2,1)
for i=1:3
plot(NMPC_time,NMPC_ukSRB_(:,i),'-','LineWidth',1.5,'Color',colors_des(i))
hold on
plot(LL_data.time,LL_data.QPforce(:,i),'--','LineWidth',1.5, 'Color',colors_achieved(i))
end
xlabel('Time(s)')
ylabel('Forces(N)')
legend('X_{FR Sol}','X_{FR LL}','Y_{FR Sol}','Y_{FR LL}','Z_{FR Sol}' ,'Z_{FR LL}');
title('NMPC GRF solution vs LL commanded (FR)')
grid on;

subplot(2,2,2)
for i=1:3
plot(NMPC_time,NMPC_ukSRB_(:,i+3),'-','LineWidth',1.5,'Color',colors_des(i))
hold on
plot(LL_data.time,LL_data.QPforce(:,i+3),'--','LineWidth',1.5, 'Color',colors_achieved(i))
end
xlabel('Time(s)')
ylabel('Forces(N)')
legend('X_{FL Sol}',' X_{FL LL}','Y_{FL Sol}','Y_{FL LL}','Z_{FL Sol}','Z_{FL LL}');
title('NMPC GRF solution vs LL commanded (FL)')
grid on;


subplot(2,2,3)
for i=1:3
plot(NMPC_time,NMPC_ukSRB_(:,i+6),'-','LineWidth',1.5,'Color',colors_des(i))
hold on
plot(LL_data.time,LL_data.QPforce(:,i+6),'--','LineWidth',1.5, 'Color',colors_achieved(i))
end
xlabel('Time(s)')
ylabel('Forces(N)')
legend('X_{RR Sol}',' X_{RR LL}','Y_{RR Sol}','Y_{RR LL}','Z_{RR Sol}','Z_{RR LL}');
title('NMPC GRF solution vs LL commanded (RR)')
grid on;

subplot(2,2,4)
for i=1:3
plot(NMPC_time,NMPC_ukSRB_(:,i+9),'-','LineWidth',1.5,'Color',colors_des(i))
hold on
plot(LL_data.time,LL_data.QPforce(:,i+9),'--','LineWidth',1.5, 'Color',colors_achieved(i))
end
xlabel('Time(s)')
ylabel('Forces(N)')
legend('X_{RL Sol}',' X_{RL LL}','Y_{RL Sol}','Y_{RL LL}','Z_{RL Sol}','Z_{RL LL}');
title('NMPC GRF solution vs LL commanded (RL)')
grid on;





%%
%{
%% save data

timestamp = datestr(datetime('now'), 'yyyy-mm-dd_hh-mm-ss');
save_path = strcat("~/onedrive/1research_legged_locomotion/a1_kinova_locomanipulation/raisim/",timestamp,'/');
if ~exist(save_path,'dir')
    mkdir(save_path)
end
disp(save_path);
save_all_figures(save_path)
save(strcat(save_path,'data.mat'),'ll_data');
% showanimation(tout_cpp,xout_cpp,uout_cpp,feet_cpp,dd,save_path,true);


%% cpp plots
plotgrfs(tout_cpp,uout_cpp)
plotintforces(tout_cpp,uout_cpp)
plotholnomconst(tout_cpp,zeros(size(xout_cpp)),xout_cpp,dd)
plotarmstates(tout_cpp,zeros(size(xout_cpp)),xout_cpp)
plotarmtorques(tout_cpp,uout_cpp)

plotcompos(tout_cpp,zeros(size(xout_cpp)),xout_cpp)
plotcomvel(tout_cpp,zeros(size(xout_cpp)),xout_cpp)
ploteulerangls(tout_cpp,zeros(size(xout_cpp)),xout_cpp)
plotdereulerangls(tout_cpp,zeros(size(xout_cpp)),xout_cpp)

plotgrfs(tout_cpp,uout_cpp)
save_all_figures(save_path)



%% data from nmpc for horizons in raisim

x_sol_all = readmatrix('/home/trec/workraj/raisim_legged/basit_nmpc/srbnmpc_a1_wll/build/tmp/sol_x_n_append.txt');
x_sol_all_reshape = reshape(x_sol_all.',32,6,[]);
x_sol_first = x_sol_all_reshape(:,1,:);
x_sol_first = reshape(x_sol_first,size(x_sol_first,1),[])';

%%  data from nmpc for state feedback in raisim
x_sol_all = readmatrix('/home/trec/workraj/raisim_legged/basit_nmpc/srbnmpc_a1_wll/build/tmp/a1_kinova_nmpc_x_log');
x_sol_first = x_sol_all(:,14:end);

%% %% plotting nmpc outputs to chck holonmic constraints
figure();
dd =[-0.1,0,0.1];
holonomic_h = zeros(size(x_sol_first,1),6);
holonomic_dh = zeros(size(x_sol_first,1),6);
theta_d = zeros(size(x_sol_first,1),3);
holonomic_h_rot_term = zeros(size(x_sol_first,1),3);

for i = 300:301%size(x_sol_first,1)
    disp(x_sol_first(i,:))
    holonomic_h(i,1:3) = (x_sol_first(i,1:3)'+rotmat(x_sol_first(i,7:9))*dd' - x_sol_first(i,13:15)')';
    holonomic_h(i,4:6) = (x_sol_first(i,7:9)'- x_sol_first(i,16:18)')';
    holonomic_dh(i,1:3) =  (skewsym(x_sol_first(i,10:12))*rotmat(x_sol_first(i,7:9))*dd')';
    holonomic_dh(i,4:6) = (a_theta(x_sol_first(i,7:9))*x_sol_first(i,10:12)' - x_sol_first(i,16:18)')';
    theta_d(i,1:3) = (a_theta(x_sol_first(i,7:9))*x_sol_first(i,10:12)')';
    holonomic_h_rot_term(i,1:3) = rotmat(x_sol_first(i,7:9))*dd';
end

subplot(3,2,1)
hold on;
plot(0.01*(1:size(x_sol_first,1)),holonomic_h(:,1)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_h(:,2)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_h(:,3)');

title('holonomic constraint h position')
ylabel('h (m)')
xlabel('t (s)')
legend('x','y','z')
grid on;

subplot(3,2,2)
hold on;
plot(0.01*(1:size(x_sol_first,1)),holonomic_dh(:,1)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_dh(:,2)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_dh(:,3)');

title('holonomic constraint dh velocity')
ylabel('dh (m/s)')
xlabel('t (s)')
legend('x','y','z')
grid on;

subplot(3,2,3)
hold on
plot(0.01*(1:size(x_sol_first,1)),holonomic_h(:,4)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_h(:,5)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_h(:,6)');

title('holonomic constraint h orientation')
ylabel('h (rad)')
xlabel('t (s)')
legend('r','p','y')
grid on;

subplot(3,2,4)
hold on
plot(0.01*(1:size(x_sol_first,1)),holonomic_dh(:,4)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_dh(:,5)');
plot(0.01*(1:size(x_sol_first,1)),holonomic_dh(:,6)');

title('holonomic constraint dh orientation')
ylabel('h (rad/s)')
xlabel('t (s)')
legend('r','p','y')
grid on;


subplot(3,2,5)
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,7)');
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,8)');
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,9)');

title('orientation')
ylabel('orientation (rad)')
xlabel('t (s)')
legend('r','p','y')
grid on;


subplot(3,2,6)
hold on
plot(0.01*(1:size(x_sol_first,1)),theta_d(:,1)');
plot(0.01*(1:size(x_sol_first,1)),theta_d(:,2)');
plot(0.01*(1:size(x_sol_first,1)),theta_d(:,3)');

title('theta_dot')
ylabel('theta_dot (rad/s)')
xlabel('t (s)')
legend('r','p','y')
grid on;


%%investigating why h and dh are non zero at starting
figure();
subplot(3,1,1)
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,1)');
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,13)','--');
hold on
plot(0.01*(1:size(x_sol_first,1)),holonomic_h_rot_term(:,1)','-.')
legend('x srb com', 'x arm base','x rotmat*dd' )
title('comparing individual terms of holonomic position constraints')
xlabel('time (s)')
ylabel('position (m)')

subplot(3,1,2)
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,2)');
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,14)','--');
hold on
plot(0.01*(1:size(x_sol_first,1)),holonomic_h_rot_term(:,2)','-.')
legend('y srb com', 'y arm base','y rotmat*dd' )
xlabel('time (s)')
ylabel('position (m)')

subplot(3,1,3)
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,3)');
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,15)','--');
hold on
plot(0.01*(1:size(x_sol_first,1)),holonomic_h_rot_term(:,3)','-.')
legend('z srb com', 'z arm base','z rotmat*dd' )
xlabel('time (s)')
ylabel('position (m)')

figure()
hold on
plot(0.01*(1:size(x_sol_first,1)),x_sol_first(:,10:12)')
legend('omega x', 'omega y','omega z' )
xlabel('time (s)')
ylabel('omega (rad/s)')

%}


%% Extras 
% 
% f3 = figure('Name','Feeedback from LL vs time')
% plot(NMPC_time,NMPC_qk_SRB(:,1))
% hold on;
% plot(NMPC_time,NMPC_qk_SRB(:,2))
% hold on;
% plot(NMPC_time,NMPC_qk_SRB(:,3))
% xlabel('Time(s)')
% ylabel('Position(m)')
% legend('X','Y','Z')
% grid on;

% f4 = figure('Name','RPY State from LL vs time')
% plot(NMPC_time,NMPC_qk_SRB(:,7))
% hold on;
% plot(NMPC_time,NMPC_qk_SRB(:,8))
% hold on;
% plot(NMPC_time,NMPC_qk_SRB(:,9))
% xlabel('Time(s)')
% ylabel('RPY Angles(rad)')
% legend('R','P','Y')
% grid on;
% 
% f12 = figure('Name','MPC Desired xyz velocity vs time')
% plot(NMPC_time,NMPC_q_des_(:,4))
% hold on;
% plot(NMPC_time,NMPC_q_des_(:,5))
% hold on;
% plot(NMPC_time,NMPC_q_des_(:,6))
% xlabel('Time(s)')
% ylabel('Desired Velocity(m/s)')
% legend('X','Y','Z')
% grid on;

% f13 = figure('Name','MPC Desired xyz position vs time')
% plot(NMPC_time,NMPC_q_des_(:,1))
% hold on;
% plot(NMPC_time,NMPC_q_des_(:,2))
% hold on;
% plot(NMPC_time,NMPC_q_des_(:,3))
% xlabel('Time(s)')
% ylabel('Desired Position(m)')
% legend('X','Y','Z')
% grid on;

% f2 = figure('Name','MPC Solution Orient vs time')
% plot(NMPC_time,NMPC_x_sol_1(:,7))
% hold on;
% plot(NMPC_time,NMPC_x_sol_1(:,8))
% hold on;
% plot(NMPC_time,NMPC_x_sol_1(:,9))
% xlabel('Time(s)')
% ylabel('Angle(m)')
% legend('R','P','Y')
% grid on;




% f16 = figure('Name','QPForce diff')
% subplot(2,2,1)
% plot(LL_data_Seq.time,QPforce_difff(:,1:3));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FR}','Y_{FR}','Z_{FR}');
% title('GRF (fdes -f) LL FR')
% grid on;
% 
% subplot(2,2,2)
% plot(LL_data_Seq.time,QPforce_difff(:,4:6));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FL}','Y_{FL}','Z_{FL}');
% title('GRF (fdes -f) LL FL')
% grid on;
% 
% subplot(2,2,3)
% plot(LL_data_Seq.time,QPforce_difff(:,7:9));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend(  'X_{RR}','Y_{RR}','Z_{RR}');
% title('GRF (fdes -f) LL RR')
% grid on;
% 
% subplot(2,2,4)
% plot(LL_data_Seq.time,QPforce_difff(:,10:12));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{RL}','Y_{RL}','Z_{RL}');
% title('GRF (fdes -f) LL RL')
% grid on;
% 
% %%
% f17 = figure('Name','QPForce Sequential')
% subplot(2,2,1)
% plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,1:3));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FR}','Y_{FR}','Z_{FR}');
% title('GRF (fdes -f) LL FR')
% grid on;
% 
% subplot(2,2,2)
% plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,4:6));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FL}','Y_{FL}','Z_{FL}');
% title('GRF (fdes -f) LL FL')
% grid on;
% 
% subplot(2,2,3)
% plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,7:9));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend(  'X_{RR}','Y_{RR}','Z_{RR}');
% title('GRF (fdes -f) LL RR')
% grid on;
% 
% subplot(2,2,4)
% plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,10:12));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{RL}','Y_{RL}','Z_{RL}');
% title('GRF (fdes -f) LL RL')
% grid on;
% 
% 
% f18 = figure('Name','QPForce Threaded')
% subplot(2,2,1)
% plot(LL_data_thread.time,LL_data_thread.QPforce(:,1:3));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FR}','Y_{FR}','Z_{FR}');
% title('GRF (fdes -f) LL FR')
% grid on;
% 
% subplot(2,2,2)
% plot(LL_data_thread.time,LL_data_thread.QPforce(:,4:6));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FL}','Y_{FL}','Z_{FL}');
% title('GRF (fdes -f) LL FL')
% grid on;
% 
% subplot(2,2,3)
% plot(LL_data_thread.time,LL_data_thread.QPforce(:,7:9));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend(  'X_{RR}','Y_{RR}','Z_{RR}');
% title('GRF (fdes -f) LL RR')
% grid on;
% 
% subplot(2,2,4)
% plot(LL_data_thread.time,LL_data_thread.QPforce(:,10:12));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{RL}','Y_{RL}','Z_{RL}');
% title('GRF (fdes -f) LL RL')
% grid on;
% 
% %%
% f19 = figure('Name','Outputs Threaded')
% subplot(2,2,1)
% plot(LL_data_thread.time,LL_data_thread.outputs(:,1:3));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FR}','Y_{FR}','Z_{FR}');
% title('GRF (fdes -f) LL FR')
% grid on;
% 
% subplot(2,2,2)
% plot(LL_data_thread.time,LL_data_thread.outputs(:,4:6));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{FL}','Y_{FL}','Z_{FL}');
% title('GRF (fdes -f) LL FL')
% grid on;
% 
% subplot(2,2,3)
% plot(LL_data_thread.time,LL_data_thread.outputs(:,7:9));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend(  'X_{RR}','Y_{RR}','Z_{RR}');
% title('GRF (fdes -f) LL RR')
% grid on;
% 
% subplot(2,2,4)
% plot(LL_data_thread.time,LL_data_thread.outputs(:,10:12));
% xlabel('time(s)');
% ylabel('GRF ERROR (fdes - f)')
% legend('X_{RL}','Y_{RL}','Z_{RL}');
% title('GRF (fdes -f) LL RL')

% grid on;

%% Filter Design Stuff
fs = 1000;
vel_x = LL_data.vel(:,1);
vel_y = LL_data.vel(:,2);
vel_z = LL_data.vel(:,3);

snr_x =  snr(vel_x,fs);
snr_y =  snr(vel_y,fs);
snr_z =  snr(vel_z,fs);

fprintf("The Signal to noise ratio for x,y,z velocities: %0.3f, %0.3f, %0.3f \n",snr_x,snr_y,snr_z);

time = LL_data.time;
n = length(vel_x);
fshift = (-n/2:n/2-1)*(fs/n);
ynoise_x = fft(vel_x);
ynoise_y = fft(vel_y);
ynoise_z = fft(vel_z);
ynoiseshift_x = fftshift(ynoise_x);
ynoiseshift_y = fftshift(ynoise_y);
ynoiseshift_z = fftshift(ynoise_z);
power_x = abs(ynoiseshift_x).^2/n;
power_y = abs(ynoiseshift_y).^2/n;
power_z = abs(ynoiseshift_z).^2/n;


f20 = figure("Name","Velocity Noise characterization by FFT")
subplot(2,2,1)
plot(fshift,power_x)
title('X Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;
subplot(2,2,2)
plot(fshift,power_y)
title('Y Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;
subplot(2,2,3)
plot(fshift,power_z)
title('Z Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;

f20 = figure("Name","Unfiltered Velocity from LL") 
subplot(2,2,1)
plot(time, vel_x);
title('X Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

subplot(2,2,2)
plot(time, vel_y);
title('Y Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

subplot(2,2,3)
plot(time, vel_z);
title('Z Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

f22 = figure("Name","Velocity Noise characterization by FFT")
subplot(2,2,1)
plot(fshift,power_x)
title('X Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;
subplot(2,2,2)
plot(fshift,power_y)
title('Y Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;
subplot(2,2,3)
plot(fshift,power_z)
title('Z Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;

%% Filter design for 3 Hz fourth order butterworth 
fc = 4; %Cutoff freq in Hz
fs = 1000; %Sampling freq in Hz
[b,a] = butter(4,fc/(fs/2),'low');
figure('Name','Filter design')
freqz(b,a,[],fs);
subplot(2,1,1)
ylim([-100 200])
fprintf("double av[3] = { %0.9f, %0.9f, %0.9f,%0.9f, %0.9f}; \n",a);
fprintf("\t\t\tdouble bv[3] = {%0.9f, %0.9f, %0.9f,%0.9f, %0.9f}; \n", b);

vel_filtered_x = filter(b,a,vel_x);
vel_filtered_y = filter(b,a,vel_y);
vel_filtered_z = filter(b,a,vel_z);

snr_filtered_x =  snr(vel_filtered_x,fs);
snr_filtered_y =  snr(vel_filtered_y,fs);
snr_filtered_z =  snr(vel_filtered_z,fs);

fprintf("The Signal to noise ratio for filtered x,y,z velocities: %0.3f, %0.3f, %0.3f \n",snr_filtered_x,snr_filtered_y,snr_filtered_z);

ynoise_filtered_x = fft(vel_filtered_x);
ynoise_filtered_y = fft(vel_filtered_y);
ynoise_filtered_z = fft(vel_filtered_z);
ynoiseshift_filtered_x = fftshift(ynoise_filtered_x);
ynoiseshift_filtered_y = fftshift(ynoise_filtered_y);
ynoiseshift_filtered_z = fftshift(ynoise_filtered_z);
power_filtered_x = abs(ynoiseshift_filtered_x).^2/n;
power_filtered_y = abs(ynoiseshift_filtered_y).^2/n;
power_filtered_z = abs(ynoiseshift_filtered_z).^2/n;


% Moving average filter
vel_filtered_x_mv = movmean(vel_x,500);
vel_filtered_y_mv = movmean(vel_y,500);
vel_filtered_z_mv = movmean(vel_z,500);


f20 = figure("Name","Unfiltered Velocity from LL") 
subplot(2,2,1)
plot(time, vel_x);
hold on
plot(time,vel_filtered_x)
hold on
plot(time,vel_filtered_x_mv)
title('X Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

subplot(2,2,2)
plot(time, vel_y);
hold on
plot(time,vel_filtered_y)
plot(time,vel_filtered_y_mv)
title('Y Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

subplot(2,2,3)
plot(time, vel_z);
hold on
plot(time,vel_filtered_z)
plot(time,vel_filtered_z_mv)
title('Z Velocity')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on;

f22 = figure("Name","Velocity Noise characterization by FFT")
subplot(2,2,1)
plot(fshift,power_x)
title('X Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;
subplot(2,2,2)
plot(fshift,power_y)
title('Y Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;
subplot(2,2,3)
plot(fshift,power_z)
title('Z Velocity Power Spectrum')
xlabel('Frequency (Hz)')
ylabel('Power')
xlim([-1 50])
grid on;


%%
%%%% Save plots
timestamp = datestr(datetime('now'), 'yyyy-mm-dd_HH-MM-SS');
save_path = strcat("~/OneDrive/1Research_legged_locomotion/A1_Kinova_locomanipulation/raisim/",timestamp,'_','Halo_MPC_in_place_trot_vel_feedback','/');
% FAIL_Halo_MPC_Standing_Walking_z_fix_r_P_0
if ~exist(save_path,'dir')
    mkdir(save_path)
end
disp(save_path);
save_all_figures(save_path)
save(strcat(save_path,'data.mat'));

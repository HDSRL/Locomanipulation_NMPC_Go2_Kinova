%% Testing only

%% Load raisim LL data
foldername = 'Jun11_822pm_Go2K';%'Test2';%'Jun10_644pm_Go2K';%%'Jun10_1255pm_Go2K50';%'May18_552pm_torquetrot_joy';%'Test';%'April24_444pm_torquetrot0.75';%'April28_655pm_torquetrot0.5';%'May05_548pm_torquetrot0.6';%'May05_420pm_torquetrot0.6';%'May05_548pm_torquetrot0.6';%;%'April7_1255pm_forward_trot';% 'April24_451pm_torquetrot0.7';% 'May05_651pm_torquetrot0.6pose1';%'Test';%'May05_548pm_torquetrot0.6';%'Test';%'May05_525pm_torquetrot0.6';%'Test';%'May05_427pm_torquetrot0.6';%'Test';%'April28_825pm_torquetrot0.2';%'Test';
filename = {[ strcat('home/trec/WorkRaj/raisim_legged/Experiment_Data/',foldername,'/LL_agent_0.txt')]};
dd = [-0.1 0 0.1]';
LL_data = parseData(filename);

colors_des = ["#0072BD","#FF0000","#EDB120"];
colors_achieved = ["#00FFFF","#D95319","#FF8800"];

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
plot(LL_data.time,LL_data.vel(:,4:6),'LineWidth',1.0);
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
plot(LL_data.time(1:end-1000),LL_data.outputs(1:end-1000,1:3),'LineWidth',1.0);
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

f30 = figure('Name','Outputs and dOutputs XYZ of LL foot')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.outputs(1:end-1000,7:12),'LineWidth',1.0);
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
plot(LL_data.time(1:end-1000),LL_data.outputs(1:end-1000,4:6),'LineWidth',1.0);
xlabel('time(s)');
ylabel('Outputs y (rad)')
title('RPY Orientation Output of LL')
legend('R','P','Y')
grid on



subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.doutputs(1:end-1000,4:6));
xlabel('time(s)');
ylabel('dOutputs y (rad/s)')
legend('R','P','Y')
title('RPY Angular Velocity Output of LL')
grid on;


f5 = figure('Name','Commanded XYZ Position/Velocity from NMPC to LL (hd/dhd)')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.hd(1:end-1000,1:3));
xlabel('time(s)');
ylabel('hd (m)')
title('Commanded XYZ Position from NMPC to LL')
legend('X','Y','Z')
grid on;

subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.dhd(1:end-1000,1:3));
xlabel('time(s)');
ylabel('dhd (m/s)')
title('Commanded XYZ Velocity from NMPC to LL')
legend('X','Y','Z')
grid on;





f6 = figure('Name','Commanded RPY Orientation/Angular Velocity from NMPC to LL (hd/dhd)')
subplot(2,1,1)
plot(LL_data.time(1:end-1000),LL_data.hd(1:end-1000,4:6));
xlabel('time(s)');
ylabel('hd (rad)')
title('Commanded RPY Orientation from NMPC to LL')
legend('R','P','Y')
grid on;

subplot(2,1,2)
plot(LL_data.time(1:end-1000),LL_data.dhd(1:end-1000,4:6));
xlabel('time(s)');
ylabel('dhd')
title('Commanded RPY Angular Velocity from NMPC to LL')
legend('R','P','Y')
grid on;


f7 = figure('Name','GRF: Desired from NMPC to LL (fdes)')
subplot(2,2,1)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,1:3));
xlabel('time(s)');
ylabel('GRF fdes (N)');
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF Desired from NMPC to LL (FR)')
grid on;

subplot(2,2,2)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,4:6));
xlabel('time(s)');
ylabel('GRF fdes (N)');
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF Desired from NMPC to LL (FL)')
grid on;


subplot(2,2,3)
plot(LL_data.time(1:end-1000),LL_data.fDes(1:end-1000,7:9));
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




f9 = figure('Name','GRF: Comparison of NMPC commanded and achieved')
for i =1:3
subplot(2,2,1)
plot(LL_data.time,LL_data.QPforce(:,i),'-','LineWidth',1.0,'Color',colors_achieved(i));
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
plot(LL_data.time,LL_data.QPforce(:,i+3),'-','LineWidth',1.0,'Color',colors_achieved(i));
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
plot(LL_data.time,LL_data.QPforce(:,i+6),'-','LineWidth',1.0,'Color',colors_achieved(i));
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
plot(LL_data.time,LL_data.QPforce(:,i+9),'-','LineWidth',1.0,'Color',colors_achieved(i));
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
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,1:3));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{FR}','HipPitch_{FR}','Knee_{FR}');
title('Motor Torques from LL (FR)')
ylim([-30,30])
grid on;

subplot(2,2,2)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,4:6));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{FL}','HipPitch_{FL}','Knee_{FL}');
title('Motor Torques from LL (FL)')
ylim([-30,30])
grid on;

subplot(2,2,3)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,7:9));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{RR}','HipPitch_{RR}','Knee_{RR}');
title('Motor Torques from LL (RR)')
ylim([-30,30])
grid on;


subplot(2,2,4)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,10:12));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{RL}','HipPitch_{RL}','Knee_{RL}');
title('Motor Torques from LL (RL)')
ylim([-30,30])
grid on;


%% NMPC Plots
%plot phase
% filename = '/home/trec/WorkRaj/raisim_legged/Basit_NMPC/SRBNMPC_A1_wLL/build/tmp/A1_KINOVA_NMPC_x_log';

filename = strcat('home/trec/WorkRaj/raisim_legged/Experiment_Data/',foldername,'/A1_KINOVA_NMPC_x_log.txt');
dd = [-0.1 0 0.1]';
NMPC_data = readmatrix(filename);
NMPC_time = NMPC_data(:,1);
NMPC_x_sol_1 = NMPC_data(:,2:13);
NMPC_qk_SRB = NMPC_data(:,14:25);
NMPC_qk_arm_ = NMPC_data(:,26:45);
NMPC_ukSRB_ = NMPC_data(:,46:57);
NMPC_ukArm_ = NMPC_data(:,58:61);
NMPC_ukLambda_ = NMPC_data(:,62:67);
NMPC_q_des_ = NMPC_data(:,68:99);



colors_des = ["#0072BD","#FF0000","#EDB120"];
colors_achieved = ["#00FFFF","#D95319","#FF8800"];
ylabels_XYZ = ["X","Y","Z"];
ylabels_RPY = ["R","P","Y"];

%% Plot data
f10 = figure('Name','Position/Velocity NMPC Solution XYZ vs NMPC Reference')

for i=1:3
    subplot(3,2,2*i-1)
    plot(NMPC_time,NMPC_x_sol_1(:,i),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i),'--','LineWidth',1.5,'Color','k')
    grid on;
    ylabel(strcat(ylabels_XYZ(i)," Position(m)"))
    title('Position: NMPC XYZ Reference vs Solution')
    legend(strcat(ylabels_XYZ(i),'_{NMPC}'),strcat(ylabels_XYZ(i),'_{ref}'));
end
xlabel('Time(s)')
ylim([-0.1,0.5])

for i=1:3
    subplot(3,2,2*i)
    plot(NMPC_time,NMPC_x_sol_1(:,i+3),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i+3),'--','LineWidth',1.5,'Color','k')
    grid on;
    ylabel(strcat(ylabels_XYZ(i)," Velocity(m/s)"))
    title('Velocity: NMPC Reference XYZ vs Solution')
    legend(strcat(ylabels_XYZ(i),'_{NMPC}'),strcat(ylabels_XYZ(i),'_{ref}'));
end
xlabel('Time(s)')
ylim([-0.1,0.5])



f11 = figure('Name','Orientation/AngularVelocity NMPC Solution RPY vs Reference')

for i=1:3
    subplot(3,2,2*i-1)
    plot(NMPC_time,NMPC_x_sol_1(:,i+6),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i+6),'--','LineWidth',1.5,'Color','k')
    ylabel(strcat(ylabels_RPY(i),' Angle(rad)'))
    legend(strcat(ylabels_RPY(i),'_{sol}'),strcat(ylabels_RPY(i),'_{ref}'))
    title('Orientation: NMPC Solution RPY vs Reference')
    grid on;
end
xlabel('Time(s)')
for i=1:3
    subplot(3,2,2*i)
    plot(NMPC_time,NMPC_x_sol_1(:,i+9),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_q_des_(:,i+9),'--','LineWidth',1.5,'Color','k')
    ylabel(strcat(ylabels_RPY(i),' Anglular velocity(rad/s)'))
    legend(strcat(ylabels_RPY(i),'_{sol}'),strcat(ylabels_RPY(i),'_{ref}'))
    title('Angular Velocity: NMPC Solution RPY vs Reference')
    grid on;
end
xlabel('Time(s)')


f12 = figure('Name','Position/Velocity NMPC Solution XYZ vs Feedback from LL')

for i=1:3
    subplot(3,2,2*i-1)
    plot(NMPC_time,NMPC_x_sol_1(:,i),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i),'--','LineWidth',1.5,'Color',colors_des(i))
    ylabel(strcat(ylabels_XYZ(i),' Position(m)'))
    legend(strcat(ylabels_XYZ(i),'_{NMPC}'),strcat(ylabels_XYZ(i),'_{fbk}'));
    title('Position: NMPC Solution XYZ vs Feedback from LL')
    grid on;
end
xlabel('Time(s)')


for i=1:3
    subplot(3,2,2*i)
    plot(NMPC_time,NMPC_x_sol_1(:,i+3),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i+3),'--','LineWidth',1.5,'Color',colors_des(i))
    ylabel(strcat(ylabels_XYZ(i),' Velocity (m/s)'))
    legend(strcat(ylabels_XYZ(i),'_{NMPC}'),strcat(ylabels_XYZ(i),'_{fbk}'));
    title('Velocity: NMPC Solution XYZ vs Feedback from LL')
    grid on;
end
xlabel('Time(s)')





f13 = figure('Name','Orientation/Angular Velocity NMPC Solution XYZ vs Feedback from LL')

for i=1:3
    subplot(3,2,2*i-1)
    plot(NMPC_time,NMPC_x_sol_1(:,i+6),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i+6),'--','LineWidth',1.5,'Color',colors_des(i))
    ylabel(strcat(ylabels_RPY(i),' Angle (rad)'))
    legend(strcat(ylabels_RPY(i),'_{NMPC}'),strcat(ylabels_RPY(i),'_{fbk}'));
    title('Orientation: NMPC Solution XYZ vs Feedback from LL')
    grid on;
end
xlabel('Time(s)')

for i=1:3
    subplot(3,2,2*i)
    plot(NMPC_time,NMPC_x_sol_1(:,i+9),'-','LineWidth',1.0,'Color',colors_achieved(i))
    hold on 
    plot(NMPC_time,NMPC_qk_SRB(:,i+9),'--','LineWidth',1.5,'Color',colors_des(i))
    ylabel(strcat(ylabels_RPY(i),' Angular Velocity (rad/s)'))
    legend(strcat(ylabels_RPY(i),'_{NMPC}'),strcat(ylabels_RPY(i),'_{fbk}'));
    title('Angular Velocity: NMPC Solution XYZ vs Feedback from LL ')
    grid on;
end
xlabel('Time(s)')




f14 = figure('Name','GRFs: NMPC Solution GRFs vs LL commanded GRFs')
subplot(2,2,1)
for i=1:3
plot(NMPC_time,NMPC_ukSRB_(:,i),'-','LineWidth',1.0,'Color',colors_des(i))
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
plot(NMPC_time,NMPC_ukSRB_(:,i+3),'-','LineWidth',1.0,'Color',colors_des(i))
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
plot(NMPC_time,NMPC_ukSRB_(:,i+6),'-','LineWidth',1.0,'Color',colors_des(i))
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
plot(NMPC_time,NMPC_ukSRB_(:,i+9),'-','LineWidth',1.0,'Color',colors_des(i))
hold on
plot(LL_data.time,LL_data.QPforce(:,i+9),'--','LineWidth',1.5, 'Color',colors_achieved(i))
end
xlabel('Time(s)')
ylabel('Forces(N)')
legend('X_{RL Sol}',' X_{RL LL}','Y_{RL Sol}','Y_{RL LL}','Z_{RL Sol}','Z_{RL LL}');
title('NMPC GRF solution vs LL commanded (RL)')
grid on;



f15 = figure('Name','Interaction Forces')
plot(NMPC_time,NMPC_ukLambda_(:,1))
hold on;
plot(NMPC_time,NMPC_ukLambda_(:,2))
hold on;
plot(NMPC_time,NMPC_ukLambda_(:,3))
xlabel('Time(s)')
ylabel('Forces(N)')
legend('X','Y','Z')
grid on;

f16 = figure('Name','Interaction Torques')
plot(NMPC_time,NMPC_ukLambda_(:,4))
hold on;
plot(NMPC_time,NMPC_ukLambda_(:,5))
hold on;
plot(NMPC_time,NMPC_ukLambda_(:,6))
xlabel('Time(s)')
ylabel('Torques(Nm)')
legend('X','Y','Z')
grid on;

f17 = figure('Name','Arm Torques')
plot(NMPC_time,NMPC_ukArm_(:,1))
hold on;
plot(NMPC_time,NMPC_ukArm_(:,2))
hold on;
plot(NMPC_time,NMPC_ukArm_(:,3));
hold on;
plot(NMPC_time,NMPC_ukArm_(:,4))
xlabel('Time(s)')
ylabel('Torques(Nm)')
legend('J1','J2','J3', 'J4')
title("NMPC Arm Joint Torques")
grid on;

f18 = figure('Name', 'Arm Position', 'Color', 'w');
hold on;

% Color set for 4 joints
colors = lines(4);

% Joint indices
joint_actual_cols = [7, 8, 9, 10];
joint_desired_cols = [19, 20, 21, 22];

% Plot each joint: actual (solid) and desired (dashed) with same color
for i = 1:4
    plot(NMPC_time, NMPC_qk_arm_(:, joint_actual_cols(i)), '-', ...
        'LineWidth', 1.8, 'Color', colors(i, :));
    plot(NMPC_time, NMPC_q_des_(:, joint_desired_cols(i)), '--', ...
        'LineWidth', 1.8, 'Color', colors(i, :));
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Position (rad)', 'FontSize', 12)
title('Arm Joint Position', 'FontSize', 14)
legend({'J1', 'J1_{des}', 'J2', 'J2_{des}', 'J3', 'J3_{des}', 'J4', 'J4_{des}'}, ...
       'Location', 'Best')
grid on;
set(gca, 'FontSize', 12)
box on;

f31 = figure('Name', 'Arm Velocity', 'Color', 'w');
hold on;

% Color set for 4 joints
colors = lines(4);

% Joint indices
joint_actual_cols = [17, 18, 19, 20];
joint_desired_cols = [29, 30, 31, 32];

% Plot each joint: actual (solid) and desired (dashed) with same color
for i = 1:4
    plot(NMPC_time, NMPC_qk_arm_(:, joint_actual_cols(i)), '-', ...
        'LineWidth', 1.8, 'Color', colors(i, :));
    plot(NMPC_time, NMPC_q_des_(:, joint_desired_cols(i)), '--', ...
        'LineWidth', 1.8, 'Color', colors(i, :));
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Velocity (rad/s)', 'FontSize', 12)
title('Arm Joint Velocity', 'FontSize', 14)
legend({'J1', 'J1_{des}', 'J2', 'J2_{des}', 'J3', 'J3_{des}', 'J4', 'J4_{des}'}, ...
       'Location', 'Best')
grid on;
set(gca, 'FontSize', 12)
box on;





%% Torque arm plots
filename = strcat('home/trec/WorkRaj/raisim_legged/Experiment_Data/',foldername,'/torque_data.txt');
dd = [-0.1 0 0.1]';
arm_torque_data = readmatrix(filename);
arm_time = arm_torque_data(:,1);
comm_torque =  arm_torque_data(:,2:5);
meas_torque = arm_torque_data(:,6:9)

f19 = figure('Name','Measured vs Commanded Arm Torques', 'Color', 'w');
hold on;

colors = lines(4);  % Get 4 distinct colors from MATLAB's default color set

% Plot measured torques (solid lines)
for i = 1:4
    plot(arm_time, meas_torque(:, i), '-', 'LineWidth', 1.8, 'Color', colors(i, :));
end

% Plot commanded torques (dashed lines)
for i = 1:4
    plot(arm_time, comm_torque(:, i), '--', 'LineWidth', 1.8, 'Color', colors(i, :));
end

xlabel('Time (s)', 'FontSize', 12)
ylabel('Torque (Nm)', 'FontSize', 12)
title('NMPC Arm Joint Torques', 'FontSize', 14)

legend({
    'J1 Measured', 'J2 Measured', 'J3 Measured', 'J4 Measured', ...
    'J1 Commanded', 'J2 Commanded', 'J3 Commanded', 'J4 Commanded'
}, 'Location', 'BestOutside')

grid on;
set(gca, 'FontSize', 12)
box on;

%% A1 Joint plots
filename = strcat('home/trec/WorkRaj/raisim_legged/Experiment_Data/',foldername,'/A1_Kinova_motorData.txt');
A1_motor_data = readmatrix(filename);
A1_motor_time = 1e-3*A1_motor_data(:,1);
A1_motor_torque =  A1_motor_data(:,2:13);
A1_motor_temp = A1_motor_data(:,14:25);

f22 = figure('Name','Motor Torques: From A1 Sensors')
subplot(2,2,1)
plot(A1_motor_time(1:end),A1_motor_torque(1:end,1:3));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{FR}','HipPitch_{FR}','Knee_{FR}');
title('Motor Torques (FR)')
ylim([-30,30])
grid on;

subplot(2,2,2)
plot(A1_motor_time(1:end),A1_motor_torque(1:end,4:6));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{FL}','HipPitch_{FL}','Knee_{FL}');
title('Motor Torques(FL)')
ylim([-30,30])
grid on;

subplot(2,2,3)
plot(A1_motor_time(1:end),A1_motor_torque(1:end,7:9));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{RR}','HipPitch_{RR}','Knee_{RR}');
title('Motor Torques(RR)')
ylim([-30,30])
grid on;


subplot(2,2,4)
plot(A1_motor_time(1:end),A1_motor_torque(1:end,10:12));
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('HipRoll_{RL}','HipPitch_{RL}','Knee_{RL}');
title('Motor Torques (RL)')
ylim([-30,30])
grid on;


f23 = figure('Name','Motor Temperature')
subplot(2,2,1)
plot(A1_motor_time(1:end),A1_motor_temp(1:end,1:3));
xlabel('time(s)');
ylabel('Motor Temperature (deg C)');
legend('HipRoll_{FR}','HipPitch_{FR}','Knee_{FR}');
title('Motor Temperature (FR)')
ylim([20,40])
grid on;

subplot(2,2,2)
plot(A1_motor_time(1:end),A1_motor_temp(1:end,4:6));
xlabel('time(s)');
ylabel('Motor Temperature (deg C)');
legend('HipRoll_{FL}','HipPitch_{FL}','Knee_{FL}');
title('Motor Temperature(FL)')
ylim([20,40])
grid on;

subplot(2,2,3)
plot(A1_motor_time(1:end),A1_motor_temp(1:end,7:9));
xlabel('time(s)');
ylabel('Motor Temperature (deg C)');
legend('HipRoll_{RR}','HipPitch_{RR}','Knee_{RR}');
title('Motor Temperature(RR)')
ylim([20,40])
grid on;


subplot(2,2,4)
plot(A1_motor_time(1:end),A1_motor_temp(1:end,10:12));
xlabel('time(s)');
ylabel('Motor Temperature (deg C)');
legend('HipRoll_{RL}','HipPitch_{RL}','Knee_{RL}');
title('Motor Temperature (RL)')
ylim([20,40])
grid on;

%% Motor torques
f20 = figure('Name','Motor Torques: Commanded by LL vs Go2 sensors')
subplot(2,2,1)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,1:3),'-','LineWidth',1.0);
hold on
plot(A1_motor_time(1:end),A1_motor_torque(1:end,1:3),'--','LineWidth',1.5);

xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('LL HipRoll_{FR}','LL HipPitch_{FR}','LL Knee_{FR}','Go2 HipRoll_{FR}','Go2 HipPitch_{FR}','Go2 Knee_{FR}');
title('Motor Torques from LL (FR)')
ylim([-30,30])
grid on;

subplot(2,2,2)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,4:6),'-','LineWidth',1.0);
hold on;
plot(A1_motor_time(1:end),A1_motor_torque(1:end,4:6),'--','LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('LL HipRoll_{FL}','LL HipPitch_{FL}','LL Knee_{FL}','Go2 HipRoll_{FL}','Go2 HipPitch_{FL}','Go2 Knee_{FL}');
title('Motor Torques from LL (FL)')
ylim([-30,30])
grid on;

subplot(2,2,3)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,7:9),'-','LineWidth',1.0);
hold on;
plot(A1_motor_time(1:end),A1_motor_torque(1:end,7:9),'--','LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('LL HipRoll_{RR}','LL HipPitch_{RR}','LL Knee_{RR}','Go2 HipRoll_{RR}','Go2 HipPitch_{RR}','Go2 Knee_{RR}');
title('Motor Torques from LL (RR)')
ylim([-30,30])
grid on;


subplot(2,2,4)
plot(LL_data.time(1:end-1000),LL_data.motorTau(1:end-1000,10:12),'-','LineWidth',1.0);
hold on;
plot(A1_motor_time(1:end),A1_motor_torque(1:end,10:12),'--','LineWidth',1.5);
xlabel('time(s)');
ylabel('Motor Torques (Nm)');
legend('LL HipRoll_{RL}','LL HipPitch_{RL}','LL Knee_{RL}','Go2 HipRoll_{RL}','Go2 HipPitch_{RL}','Go2 Knee_{RL}');
title('Motor Torques from LL (RL)')
ylim([-30,30])
grid on;



%% Save plots
timestamp = datestr(datetime('now'), 'yyyy-mm-dd_HH-MM-SS');
save_path = strcat("~/NASDrive/1Research_legged_locomotion/A1_Kinova_locomanipulation/raisim/",timestamp,'_','FAIL_Halo_MPC_in_place_trot_exps_roll_pitch_fix','/');
% FAIL_Halo_MPC_Standing_Walking_z_fix_r_P_0
if ~exist(save_path,'dir')
    mkdir(save_path)
end
disp(save_path);
save_all_figures(save_path)
save(strcat(save_path,'data.mat'));


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

f16 = figure('Name','QPForce diff')
subplot(2,2,1)
plot(LL_data_Seq.time,QPforce_difff(:,1:3));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF (fdes -f) LL FR')
grid on;

subplot(2,2,2)
plot(LL_data_Seq.time,QPforce_difff(:,4:6));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF (fdes -f) LL FL')
grid on;

subplot(2,2,3)
plot(LL_data_Seq.time,QPforce_difff(:,7:9));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend(  'X_{RR}','Y_{RR}','Z_{RR}');
title('GRF (fdes -f) LL RR')
grid on;

subplot(2,2,4)
plot(LL_data_Seq.time,QPforce_difff(:,10:12));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{RL}','Y_{RL}','Z_{RL}');
title('GRF (fdes -f) LL RL')
grid on;

%%
f17 = figure('Name','QPForce Sequential')
subplot(2,2,1)
plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,1:3));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF (fdes -f) LL FR')
grid on;

subplot(2,2,2)
plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,4:6));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF (fdes -f) LL FL')
grid on;

subplot(2,2,3)
plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,7:9));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend(  'X_{RR}','Y_{RR}','Z_{RR}');
title('GRF (fdes -f) LL RR')
grid on;

subplot(2,2,4)
plot(LL_data_Seq.time,LL_data_Seq.QPforce(:,10:12));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{RL}','Y_{RL}','Z_{RL}');
title('GRF (fdes -f) LL RL')
grid on;


f18 = figure('Name','QPForce Threaded')
subplot(2,2,1)
plot(LL_data_thread.time,LL_data_thread.QPforce(:,1:3));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF (fdes -f) LL FR')
grid on;

subplot(2,2,2)
plot(LL_data_thread.time,LL_data_thread.QPforce(:,4:6));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF (fdes -f) LL FL')
grid on;

subplot(2,2,3)
plot(LL_data_thread.time,LL_data_thread.QPforce(:,7:9));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend(  'X_{RR}','Y_{RR}','Z_{RR}');
title('GRF (fdes -f) LL RR')
grid on;

subplot(2,2,4)
plot(LL_data_thread.time,LL_data_thread.QPforce(:,10:12));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{RL}','Y_{RL}','Z_{RL}');
title('GRF (fdes -f) LL RL')
grid on;

%%
f19 = figure('Name','Outputs Threaded')
subplot(2,2,1)
plot(LL_data_thread.time,LL_data_thread.outputs(:,1:3));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FR}','Y_{FR}','Z_{FR}');
title('GRF (fdes -f) LL FR')
grid on;

subplot(2,2,2)
plot(LL_data_thread.time,LL_data_thread.outputs(:,4:6));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{FL}','Y_{FL}','Z_{FL}');
title('GRF (fdes -f) LL FL')
grid on;

subplot(2,2,3)
plot(LL_data_thread.time,LL_data_thread.outputs(:,7:9));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend(  'X_{RR}','Y_{RR}','Z_{RR}');
title('GRF (fdes -f) LL RR')
grid on;

subplot(2,2,4)
plot(LL_data_thread.time,LL_data_thread.outputs(:,10:12));
xlabel('time(s)');
ylabel('GRF ERROR (fdes - f)')
legend('X_{RL}','Y_{RL}','Z_{RL}');
title('GRF (fdes -f) LL RL')
grid on;


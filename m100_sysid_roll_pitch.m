clear all
path(path, './read_bags');
path(path, './helper_functions');

%% PARAMETERS
delay=[]; NaN; % [] menas no time delay or NaN for enabling delay estimation.
% Assume 1st  order system  
np = 1;
nz = 0;
use_one_file = false;
bag1_ratio = 0.5;% 50% for bag 1, 50% for bag 2, ignore if use_one_file is false
bagfile_exp1 =  'bravo_roll.bag';
bagfile_exp2 =  'bravo_pitch.bag'; % ignore if use_one_file is true

topic_imu = '/matrice/imu';
topic_vcdata = '/matrice/command/roll_pitch_yawrate_thrust';
topic_rc = '/matrice/rc';

bag1_start = 23;
bag2_start = 23;
bag1_end = 10;
bag2_end = 10;

if (use_one_file)
    if ((bag1_ratio <= 0) || (bag1_ratio >= 1))
        bag1_ratio = 0.5;
    end
    bag1_select = rosbag(bagfile_exp1);
    mid_time = bag1_select.StartTime + ...
        (bag1_select.EndTime - bag1_select.StartTime) * bag1_ratio;
    rc_exp_1 = select(bag1_select,'Time',...
        [bag1_select.StartTime mid_time],'Topic', topic_rc);
    rc_exp_2 = select(bag1_select,'Time',...
        [mid_time bag1_select.EndTime],'Topic', topic_rc);
    imu_exp_1 = select(bag1_select,'Time',...
        [bag1_select.StartTime mid_time],'Topic', topic_imu);
    imu_exp_2 = select(bag1_select,'Time',...
        [mid_time bag1_select.EndTime],'Topic', topic_imu);
else
    bag1_select = rosbag(bagfile_exp1);
    bag2_select = rosbag(bagfile_exp2);
    rc_exp_1 = select(bag1_select,'Topic', topic_rc);
    rc_exp_2 = select(bag2_select,'Topic', topic_rc);
    imu_exp_1 = select(bag1_select,'Topic', topic_imu);
    imu_exp_2 = select(bag2_select,'Topic', topic_imu);    
end

%% Prepare datasets
Experiment1.IMU = readImu_matlab(imu_exp_1);
Experiment1.RCData = RC2RolllPitchYawRateThrust(rc_exp_1);
Experiment2.IMU = readImu_matlab(imu_exp_2);
Experiment2.RCData = RC2RolllPitchYawRateThrust(rc_exp_2);

% Write the quaternions properly

Experiment1.IMU.q = [Experiment1.IMU.q(4,:); Experiment1.IMU.q(1,:); ...
    Experiment1.IMU.q(2,:); Experiment1.IMU.q(3,:)];
Experiment2.IMU.q = [Experiment2.IMU.q(4,:); Experiment2.IMU.q(1,:); ...
    Experiment2.IMU.q(2,:); Experiment2.IMU.q(3,:)];

%time from 0
Experiment1.RCData.t = Experiment1.RCData.t - Experiment1.RCData.t(1);
Experiment1.IMU.t = Experiment1.IMU.t - Experiment1.IMU.t(1);


Experiment2.RCData.t = Experiment2.RCData.t - Experiment2.RCData.t(1);
Experiment2.IMU.t = Experiment2.IMU.t - Experiment2.IMU.t(1);

%========================================================
%RCDATA interpolation with respect to IMU,
%Exp1
%========================================================

cmd_roll_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.roll,Experiment1.IMU.t,'spline');
cmd_pitch_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.pitch,Experiment1.IMU.t,'spline');
cmd_yawrate_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.yaw_rate,Experiment1.IMU.t,'spline');
cmd_thrust_intp=interp1(Experiment1.RCData.t,Experiment1.RCData.thrust,Experiment1.IMU.t,'spline');

Experiment1.IMU.q=[Experiment1.IMU.q(1,:);Experiment1.IMU.q(2,:);Experiment1.IMU.q(3,:);Experiment1.IMU.q(4,:)];

Experiment1.RCData.roll=cmd_roll_intp;
Experiment1.RCData.pitch=cmd_pitch_intp;
Experiment1.RCData.yaw_rate=cmd_yawrate_intp;
Experiment1.RCData.verti_vel=cmd_thrust_intp;

Experiment1.RCData.t=zeros(1,size(cmd_roll_intp,2));
Experiment1.RCData.t=Experiment1.IMU.t;

%Exp2

cmd_roll_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.roll,Experiment2.IMU.t,'spline');
cmd_pitch_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.pitch,Experiment2.IMU.t,'spline');
cmd_yawrate_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.yaw_rate,Experiment2.IMU.t,'spline');
cmd_thrust_intp=interp1(Experiment2.RCData.t,Experiment2.RCData.thrust,Experiment2.IMU.t,'spline');

Experiment2.IMU.q=[Experiment2.IMU.q(1,:);Experiment2.IMU.q(2,:);Experiment2.IMU.q(3,:);Experiment2.IMU.q(4,:)];

Experiment2.RCData.roll=cmd_roll_intp;
Experiment2.RCData.pitch=cmd_pitch_intp;
Experiment2.RCData.yaw_rate=cmd_yawrate_intp;
Experiment2.RCData.verti_vel=cmd_thrust_intp;

Experiment2.RCData.t=zeros(1,size(cmd_roll_intp,2));
Experiment2.RCData.t=Experiment2.IMU.t;


Experiment1.rpy_imu = quat2rpy(Experiment1.IMU.q);
Experiment2.rpy_imu = quat2rpy(Experiment2.IMU.q);

% For DJI M100 platform.
% Please have a look "DJI_M100_regression" for more detail.

Experiment1.roll_cmd  = Experiment1.RCData.roll;
Experiment1.pitch_cmd  = Experiment1.RCData.pitch;
% Experiment1.thrust_cmd  = (Experiment1.RCData.verti_vel-1024)...
%     *k_thrust;%stick velocity command.


Experiment2.roll_cmd  = Experiment2.RCData.roll;
Experiment2.pitch_cmd  = Experiment2.RCData.pitch;
% Experiment2.thrust_cmd  = (Experiment2.RCData.verti_vel-1024)...
%     *k_thrust;

%%
% *Plot position from experiment 1*
close all;
%%
% *Plot attitude from experiment 1*
figure;
title('Experiment 1 Data');
subplot(2,1,1);
plot(Experiment1.IMU.t, Experiment1.rpy_imu(1,:)*180/pi, ...
    Experiment1.RCData.t, Experiment1.roll_cmd*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
legend('y','y_{ref}');
ylabel('roll [deg]');
title('roll from IMU');

subplot(2,1,2);
plot(Experiment1.IMU.t, Experiment1.rpy_imu(2,:)*180/pi, ...
    Experiment1.RCData.t, Experiment1.pitch_cmd*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('pitch [deg]');
title('pitch from IMU');


%%
% *Plot position from experiment 2*
figure;
%%
% *Plot attitude from experiment 2*
title('Experiment 2 Data');
subplot(2,1,1);
plot(Experiment2.IMU.t, Experiment2.rpy_imu(1,:)*180/pi, ...
    Experiment2.RCData.t, Experiment2.roll_cmd*180/pi,...
    'g--', 'linewidth', 2);

legend('y','y_{ref}');
xlabel('time');
ylabel('roll [deg]');
title('roll from IMU');

subplot(2,1,2);
plot(Experiment2.IMU.t, Experiment2.rpy_imu(2,:)*180/pi, ...
    Experiment2.RCData.t, Experiment2.pitch_cmd*180/pi, ...
    'g--', 'linewidth', 2);

xlabel('time');
ylabel('pitch [deg]');
title('pitch from IMU');

%% Identification of roll system


%% The length of data may vary.
%Experiment1.t = (Experiment1.IMU.t + Experiment1.RCData.t)/2;
Experiment1.t = Experiment1.RCData.t;
Experiment1.u1 = Experiment1.roll_cmd;
Experiment1.y1 = Experiment1.rpy_imu(1,:);
Experiment1.Ts = mean(diff(Experiment1.t));


%%
% *get rid of first and last 10 seconds (to remove ground and transient effects)*
Experiment1.u1 = Experiment1.u1(Experiment1.t > bag1_start & ...
    Experiment1.t < Experiment1.t(end) - bag1_end);
Experiment1.y1 = Experiment1.y1(Experiment1.t > bag1_start &...
    Experiment1.t < Experiment1.t(end) - bag1_end);
%Experiment1.t = Experiment1.t(Experiment1.t>10 & Experiment1.t < Experiment1.t(end)-10);


[roll,lag_roll]=xcorr(Experiment1.u1,Experiment1.y1);

[~,I_roll]=max(roll);
time_sync_roll=lag_roll(I_roll);


roll_data1 = iddata(Experiment1.y1',Experiment1.u1',Experiment1.Ts, ...
    'ExperimentName', 'RollSysID_1', 'InputName','roll_{cmd}', ...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');

roll_data1 = detrend(roll_data1);


%% The length of data may vary.
Experiment2.t = Experiment2.RCData.t;
Experiment2.u1 = Experiment2.roll_cmd;
Experiment2.y1 = Experiment2.rpy_imu(1,:);
Experiment2.Ts = mean(diff(Experiment2.t));


%get rid of first and last 10 seconds (to remove ground and transient effects)
Experiment2.u1 = Experiment2.u1(Experiment2.t > bag2_start &...
    Experiment2.t < Experiment2.t(end) - bag2_end);
Experiment2.y1 = Experiment2.y1(Experiment2.t > bag2_start &...
    Experiment2.t < Experiment2.t(end) - bag2_end);
%Experiment2.t = Experiment2.t(Experiment2.t>10 & Experiment2.t < Experiment2.t(end)-10);

roll_data2 = iddata(Experiment2.y1',Experiment2.u1',Experiment2.Ts,...
    'ExperimentName', 'RollSysID_2', 'InputName','roll_{cmd}',...
    'OutputName','roll', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');

roll_data2 = detrend(roll_data2);   


%%
% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3

%Generate model using Experiment1 and validate the model with Experiment2
roll_estimated_tf1 = tfest(roll_data1,np, nz,delay);

[~, fit1, ~] = compare(roll_data2, roll_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
roll_estimated_tf2 = tfest(roll_data2,np, nz,delay);

[~, fit2, ~] = compare(roll_data1, roll_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    roll_estimated_tf = roll_estimated_tf1;
    disp('The roll model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(roll_data2, roll_estimated_tf1);
    disp(strcat('The roll model fits the validation data with **',...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    roll_estimated_tf = roll_estimated_tf2;
    disp('The roll model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(roll_data1, roll_estimated_tf2);
    disp(strcat('The roll model fits the validation data with **',...
        num2str(fit2), '** %'));
end


%% Identification of Pitch System
Experiment1.u2 = Experiment1.pitch_cmd;
Experiment1.y2 = Experiment1.rpy_imu(2,:);

%get rid of first and last 10 seconds (to remove ground and transient effects)
Experiment1.u2 = Experiment1.u2(Experiment1.t > bag1_start &...
    Experiment1.t < Experiment1.t(end) - bag1_end);
Experiment1.y2 = Experiment1.y2(Experiment1.t > bag1_start &...
    Experiment1.t < Experiment1.t(end) - bag1_end);
Experiment1.t = Experiment1.t(Experiment1.t > bag1_start &...
    Experiment1.t < Experiment1.t(end) - bag1_end);

pitch_data1 = iddata(Experiment1.y2',Experiment1.u2',Experiment1.Ts,...
    'ExperimentName', 'PitchSysID_1', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad',...
    'TimeUnit','Second');

%remove any trend in the data
pitch_data1 = detrend(pitch_data1);

Experiment2.u2 = Experiment2.pitch_cmd;
Experiment2.y2 = Experiment2.rpy_imu(2,:);


%get rid of first and last 10 seconds (to remove ground and transient effects)
Experiment2.u2 = Experiment2.u2(Experiment2.t > bag2_start &...
    Experiment2.t < Experiment2.t(end) - bag2_end);
Experiment2.y2 = Experiment2.y2(Experiment2.t > bag2_start &...
    Experiment2.t < Experiment2.t(end) - bag2_end);
Experiment2.t = Experiment2.t(Experiment2.t > bag2_start &...
    Experiment2.t < Experiment2.t(end) - bag2_end);

pitch_data2 = iddata(Experiment2.y2',Experiment2.u2',Experiment2.Ts, ...
    'ExperimentName', 'PitchSysID_2', 'InputName','pitch_{cmd}',...
    'OutputName','pitch', 'InputUnit','rad', 'OutputUnit','rad', ...
    'TimeUnit','Second');
pitch_data2 = detrend(pitch_data2);   


%%
% *At this point we have 3 options!*
% 
% # Estimate a model from both experiments - but cannot validate it on independent dataset
% # Estimate a model from Exp1 and validate it on data from Exp2
% # Estimate a model from Exp2 and validate it on data from Exp1
%For now we choose the best model from options 2 and 3
  
%Assume 1st order system
%np = 1;
%nz = 0;

%Generate model using Experiment1 and validate the model with Experiment2
pitch_estimated_tf1 = tfest(pitch_data1,np, nz,delay);

[~, fit1, ~] = compare(pitch_data2, pitch_estimated_tf1);

%Generate model using Experiment2 and validate the model with Experiment1
pitch_estimated_tf2 = tfest(pitch_data2,np, nz,delay);

[~, fit2, ~] = compare(pitch_data1, pitch_estimated_tf2);

if fit1>fit2
    %We pick the first Identification
    pitch_estimated_tf = pitch_estimated_tf1;
    disp('The pitch model is estimated using experiment 1 and validated on data from experiment 2');
    figure;
    compare(pitch_data2, pitch_estimated_tf1);
    disp(strcat('The pitch model fits the validation data with **', ...
        num2str(fit1), '** %'));
else
    %We pick the second Identification
    pitch_estimated_tf = pitch_estimated_tf2;
    disp('The pitch model is estimated using experiment 2 and validated on data from experiment 1');
    figure;
    compare(pitch_data1, pitch_estimated_tf2);
    disp(strcat('The pitch model fits the validation data with **', ...
        num2str(fit2), '** %'));
end



%% Estimate the Whole System as 2-input 2-output MIMO System
% *The purpose here is to see of there is coupling*

Experiment2.Ts = Experiment1.Ts;    
Data1 = iddata([Experiment1.y1', Experiment1.y2'], ...
    [Experiment1.u1', Experiment1.u2'], Experiment1.Ts, ...
    'ExperimentName', 'RollPitchSysID_1', ...
    'InputName',{'roll_{cmd}','pitch_{cmd}'},...
    'OutputName',{'roll','pitch'}', ...
    'InputUnit',{'rad', 'rad'},...
    'OutputUnit',{'rad', 'rad'},...
    'TimeUnit','Second');


                          
Data2 = iddata([Experiment2.y1', Experiment2.y2'], ...
    [Experiment2.u1', Experiment2.u2'], Experiment2.Ts, ...
    'ExperimentName', 'RollPitchSysID_2', ...
    'InputName',{'roll_{cmd}','pitch_{cmd}'},...
    'OutputName',{'roll','pitch'}', ...
    'InputUnit',{'rad', 'rad'},...
    'OutputUnit',{'rad', 'rad'}, ...
    'TimeUnit','Second');


MergedData = merge(Data1, Data2);

%np = 1;
%nz = 0;
Full_estimated_tf = tfest(MergedData, np,nz);

%figure;
%bodemag(Full_estimated_tf);


%%% Estimated Transfer Functions

disp('Roll estimated transfer function is: ');
tf(roll_estimated_tf)

if(np==1)
    roll_params=getpvec(roll_estimated_tf);
    roll_gain=roll_params(1)/roll_params(2);
    roll_tau=1/roll_params(2);
    fprintf('roll tau=%.3f, gain=%.3f\n',roll_tau,roll_gain);
elseif(np==2)
    roll_params=getpvec(roll_estimated_tf);
    roll_omega=sqrt(roll_params(3));
    roll_gain=roll_params(1)/roll_params(3);
    roll_damping=roll_params(2)/(2*roll_omega);
    fprintf('roll omega=%.3f, gain=%.3f damping=%.3f\n',roll_omega,roll_gain,roll_damping);
end



% figure('Name','System analysis (roll)');
% subplot(311);
% bode(roll_estimated_tf); grid;
% title('Roll bode plot');
% 
% subplot(312);
% %rlocusplot(roll_estimated_tf); grid;
% title('Roll RootLucas plot');
% 
% subplot(313);
% step(roll_estimated_tf); grid;
% title('Roll step response plot');

disp('Pitch estimated transfer function is: ');
tf(pitch_estimated_tf)

if(np==1)
    pitch_params=getpvec(pitch_estimated_tf);
    pitch_gain=pitch_params(1)/pitch_params(2);
    pitch_tau=1/pitch_params(2);
    fprintf('pitch tau=%.3f, gain=%.3f\n',pitch_tau, pitch_gain);
elseif(np==2)
    pitch_params=getpvec(pitch_estimated_tf);
    pitch_omega=sqrt(pitch_params(3));
    pitch_gain=pitch_params(1)/pitch_params(3);
    pitch_damping=pitch_params(2)/(2*pitch_omega);
    fprintf('pitch omega=%.3f, gain=%.3f damping=%.3f\n',pitch_omega,pitch_gain,pitch_damping);
end



% figure('Name','System analysis (pitch)');
% subplot(311);
% bode(pitch_estimated_tf); grid;
% title('Pitch bode plot');
% 
% subplot(312);
% %rlocusplot(pitch_estimated_tf); grid;
% title('Pitch RootLucas plot');
% 
% subplot(313);
% step(pitch_estimated_tf); grid;
% title('Pitch step response plot');

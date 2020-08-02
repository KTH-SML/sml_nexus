%% ============================= Description ==============================
% This script logs the velocity obtained when sending different PWM
% commands to each motor of the robot. It used to get the motor's behavior
% and write a proper control law.
%
% To use it, setup the robot with the proper arduino firmware so that it 
% can receive PWM commands and broadcast velocity of each wheel over ROS.
% Running the script will run the robot forward faster and faster until it
% has logged data all over the PWM duty cycle.
%% ========================================================================

clear;
clear global;

%% ================================
%          Connect to ROS
%  ================================
% clear all
setenv('ROS_MASTER_URI','http://12.0.4.6:11311');
% setenv('ROS_HOSTNAME','12.0.10.245');
setenv('ROS_IP','12.0.10.255');
rosshutdown
pause(0.5);
rosinit('12.0.4.6',11311);

%------------------
% Setup publishers
%------------------
% Command publisher
[pwm_pub, pwm_message] = rospublisher('/cmd_pwm', 'std_msgs/Float32MultiArray');

%-------------------
% Setup subscribers
%-------------------
% Feedback publisher
vel_sub = rossubscriber('/velocity');
vel_message = receive(vel_sub);

% Set rate
freq = 50;
r = rosrate(freq);

n=52;
wait_time = 4; %in seconds
commands = linspace(0, 255, n);

log_vel = [vel_message];
log_pwm = zeros(n,1);

pwm_message.Data = [0, 0, 0, 0];
send(pwm_pub, pwm_message);

for i=1:numel(commands)
   fprintf("\nGetting velocity for PWM command: %d", commands(i));
   pwm_message.Data = [commands(i), commands(i), commands(i), commands(i)];
   log_pwm(i,1) = commands(i);
   log_pwm(i,1)
   tic;
   while toc<wait_time
       send(pwm_pub, pwm_message);
       waitfor(r);
   end
   vel_message = receive(vel_sub);
   log_vel(i) = vel_message;
end

fprintf("\nStopping and saving data...");
pwm_message.Data = [0, 0, 0, 0];
send(pwm_pub, pwm_message);

save log_nexus_pwm_adashield_12V log_vel log_pwm;

fprintf("\nData saved!");

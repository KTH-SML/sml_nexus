%% ============================= Description ==============================
% Run first the nexus_robot_pwm_logger.m script to get the PWM/velocity
% data from the real robot.
% 
% This script generates a polynomial function for converting a velocity
% command into a PWM command to apply to the motor. The polynomial 
% coefficients for each motor (pUL, pUR, pLL, and pLR) obtained here need 
% to be written to the nexus robot config file.
%   
%   pUL = UL motor (motor 2)
%   pUR = UR motor (motor 3)
%   pLL = LL motor (motor 1)
%   pLR = LR motor (motor 4)
%% ========================================================================

start_moving = [false false false false];
start_moving_index = [0 0 0 0];
vel_wheel = zeros(4,numel(log_vel));
pwm_wheel = zeros(4,numel(log_vel));

%For each wheel
  for j=1:4
    vel_wheel(j,1) = abs(log_vel(1).Data(j));
  end
for i=2:numel(log_vel)
    %For each wheel
    for j=1:4
        vel_wheel(j,i) = abs(log_vel(i).Data(j));
        pwm_wheel(j,i) = pwm_wheel(j,i-1)+5;
        if start_moving(j) == false
            if vel_wheel(j,i) > 0
                start_moving(j) = true;
                start_moving_index(j) = i;
            end
        end
    end
end

%X1 = [ones(numel(log_pwm),1),log_pwm];
%b1 = log_pwm\transpose(vel_wheel(1,:));

%Create polynomial fitting
pUL = polyfit(transpose(vel_wheel(1,start_moving_index(1):end)), log_pwm(start_moving_index(1):end), 4);
pUR = polyfit(transpose(vel_wheel(2,start_moving_index(2):end)), log_pwm(start_moving_index(2):end), 4);
pLL = polyfit(transpose(vel_wheel(3,start_moving_index(3):end)), log_pwm(start_moving_index(3):end), 4);
pLR = polyfit(transpose(vel_wheel(4,start_moving_index(4):end)), log_pwm(start_moving_index(4):end), 4);
wheel_1_command_polynomial = polyval(pUL, vel_wheel(1,:));
wheel_2_command_polynomial = polyval(pUR, vel_wheel(2,:));
wheel_3_command_polynomial = polyval(pLL, vel_wheel(3,:));
wheel_4_command_polynomial = polyval(pLR, vel_wheel(4,:));

hold on;
plot(vel_wheel(1,start_moving_index(1):end), log_pwm(start_moving_index(1):end), '-o');
plot(vel_wheel(2,start_moving_index(2):end), log_pwm(start_moving_index(2):end), '-o');
plot(vel_wheel(3,start_moving_index(3):end), log_pwm(start_moving_index(3):end), '-o');
plot(vel_wheel(4,start_moving_index(4):end), log_pwm(start_moving_index(4):end), '-o');
%plot(log_pwm, log_pwm*b1, '--');

plot(vel_wheel(1,:), wheel_1_command_polynomial);
plot(vel_wheel(2,:), wheel_2_command_polynomial);
plot(vel_wheel(3,:), wheel_3_command_polynomial);
plot(vel_wheel(4,:), wheel_4_command_polynomial);

title('Velocity/PWM command for 12V')
hold off;
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
    end
end

%X1 = [ones(numel(log_pwm),1),log_pwm];
%b1 = log_pwm\transpose(vel_wheel(1,:));

%Create polynomial fitting
p1 = polyfit(transpose(vel_wheel(1,:)), log_pwm, 6);
p2 = polyfit(transpose(vel_wheel(2,:)), log_pwm, 6);
p3 = polyfit(transpose(vel_wheel(3,:)), log_pwm, 6);
p4 = polyfit(transpose(vel_wheel(4,:)), log_pwm, 6);
wheel_1_command_polynomial = polyval(p1, vel_wheel(1,:));
wheel_2_command_polynomial = polyval(p2, vel_wheel(2,:));
wheel_3_command_polynomial = polyval(p3, vel_wheel(3,:));
wheel_4_command_polynomial = polyval(p4, vel_wheel(4,:));

hold on;
plot(vel_wheel(1,:), log_pwm, '-o');
plot(vel_wheel(2,:), log_pwm, '-o');
plot(vel_wheel(3,:), log_pwm, '-o');
plot(vel_wheel(4,:), log_pwm, '-o');
%plot(log_pwm, log_pwm*b1, '--');

plot(vel_wheel(1,:), wheel_1_command_polynomial);
plot(vel_wheel(2,:), wheel_2_command_polynomial);
plot(vel_wheel(3,:), wheel_3_command_polynomial);
plot(vel_wheel(4,:), wheel_4_command_polynomial);

title('Velocity/PWM command for 12V')
hold off;
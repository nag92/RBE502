clear all;
close all;
clc;

m1 = 10;
m2 = 5;
time = 10;
% intial and final positions in meters
[x2_i(1,1),x2_i(2,1)] = IK(1+sind(30),1+cos(30))
[x2_f(1,1),x2_f(2,1)] = IK(0,2)

theta1_path = planTraj(x2_i(1),x2_f(1),time);
theta2_path = planTraj(x2_i(2),x2_f(2),time);


sim('Problem1_model')
secs = linspace(0,10, length(q));

figure(1)
hold on;
plot(secs, qdd(:,:))
title('Joint acceleration');
ylabel('accel (rad/(s*s))')
xlabel('time (s)')
legend('qdd1 desired','qdd2 desired','qdd1 actual','qdd2 actual')
hold off


figure(2)
hold on;
plot(secs, qd(:,:))
title('Joint velocity');
ylabel('velocity (rad/s)')
xlabel('time (s)')
legend('qd1 desired','qd2 desired','qd1 actual','qd2 actual')
hold off


figure(3)
hold on;
plot(secs, q(:,:))
title('Joint angle');
ylabel('position (rad)')
xlabel('time (s)')
legend('q1 desired','q2 desired','q1 actual','q2 actual')
hold off

figure(4)
hold on;
plot(secs, O(:,:))
title('Joint position');
ylabel('position (m)')
xlabel('time (s)')
legend('O1_x','O1_y','O2_x','O2_y')
hold off



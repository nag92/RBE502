%% Problem 2 


%% A
clear all;
close all;
clc;

m1 = .05;
m2 = .05;

% intial and final positions in meters
x2_i = IK(;
x2_f = [-0.3;0.45];

x_path = planTraj(x2_i(1),x2_f(1), 3);
y_path = planTraj(x2_i(2),x2_f(2),3);

p = [];
v = [];
a = [];
%[x,xd,xdd] = getKin(x_path,5);
%[y,yd,ydd] = getKin(y_path,5)
% for t = 0:1:5
%     [x,xd,xdd] = getKin(x_path,t);
%     [y,yd,ydd] = getKin(y_path,t);
%     tau = [t;t];
%     state = A*state + B*tau
%     [theta1, theta2] = IK(x,y);
%     p = [ p, y];
%     v = [ v, sqrt( xd*xd + yd*yd)];
%     a = [ a, sqrt( xdd*xdd + ydd*ydd)];
% end


%sim('problem2_1')
disp('sddddddddddddddddddddddddddddddddddddddddd')
%run = 0:1:5 
%plot(run,p,'bd-')


%% Problem 2 


%% A
clear all;
close all;
clc;
t = 1;
m1 = .05;
m2 = .05;
A = [ 1 t 0 0;
      0 1 0 0;
      0 0 1 t;
      0 0 0 1];
  B = [ t^2/(2) 0 ; t/1, 0;0, t^2/(2);0, t/1]
  C = eye(4);
  D = B*0;
  state = [0;0;0;0]

% intial and final positions in meters
x2_i = [0.3;0.45];
x2_f = [-0.3;0.45];
time = 5;%s

x_path = planTraj(x2_i(1),x2_f(1),time);
y_path = planTraj(x2_i(2),x2_f(2),time);

p = [];
v = [];
a = [];
[x,xd,xdd] = getKin(x_path,5);
    [y,yd,ydd] = getKin(y_path,5)
for t = 0:1:5
    [x,xd,xdd] = getKin(x_path,t);
    [y,yd,ydd] = getKin(y_path,t);
    tau = [t;t];
    state = A*state + B*tau
    [theta1, theta2] = IK(x,y);
    p = [ p, y];
    v = [ v, sqrt( xd*xd + yd*yd)];
    a = [ a, sqrt( xdd*xdd + ydd*ydd)];
end

hold on;

sim('problem2')
run = 0:1:5 
plot(run,p,'bd-')


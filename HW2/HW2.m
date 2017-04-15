%% Nathaniel Goldfarb
% HW2
%%
clear all
close all
clc

%% Problem 1a
%
% $$m_1 = e^{0t}$
%
% $$m_2 = e^{-3t}$


%% Problem 1a
G = tf( [200], [1,3,0])
%%
pole(G)


%% Problem 1b

pzmap(G)
%% 
% Since on poles is non-zero, and one is less then zero
% the system is said to maringal stable

%% Problem 1c1 and 1c2 
% (x+4)^3 = x^3 + 12x^2 + 48x + 64
B = [ 64; 48; 12; 1]
A = [0 0 200 0;...
     3 0 0 200;...
     1 3 0 0;...
     0 1 0 0]
x = A\B

C = tf([ x(4) x(3)], [ x(2),x(3)])

%% Problem 1c3

impulseplot(C*G)


%% Problem 2
%
% Let, $$\beta = (Js^2 + Bs)^{-1}$
%
% Let, $$\alpha = (K_p + K_i/s)$
%
% Let, $$\gamma = 1 + \alpha \beta +K_d \beta s$
%
% $$\theta = \frac{\alpha \beta}{\gamma}\theta^{d} - \frac{\beta}{\gamma}D$




%% Problem 3a

Kd = 20;
Kp = 30;
J = 2;
B = 1
omega = 1;
D = 1;
s = tf('s');

%%
% The model and controller are:

A  = 1/(J*s^2 + (B+Kd)*s + Kp);

H = A*((Kp*omega) - D)


%%
% The poles of the transfer function are:

figure(2)
step(H)

%%
% Based on the transfer fucntion The damping ratio and natural frequnecy
% is:

wn = sqrt(15)
zeta = (.5*21)/(2*wn)
a = 1;
b = 2*zeta*wn;
c = wn*wn;
%%

if b*b > 4*a*c
    disp('over Damped')
elseif b*b < 4*a*c
    disp('under damped')
else
    disp('critial damped')
end

%% Problem 3b
% The values for the PD control are
Kd_new = 79;
Kp_new = 600;
A  = 1/(J*s^2 + (B+Kd_new)*s + Kp_new);
H2 = A*((Kp*omega) - D)
%% Proble 3c

step(20*H2)

%% Problem 3d
%
% $$\lim_{x\to\infty} s\frac{40}{s} \frac{-1}{2s^{2}+80s +600}=-40/600$
% 

A  = 1/(J*s^2 + (B+Kd_new)*s + Kp_new);
D = 40;
omega = 0;
H3 = A*((Kp*omega) - D)
stepplot(H3)

%% Problem 4a
% using the RH the following bounaries are found
%
% $$1+K_{d} > 0$
%
% $$K_p(1+K_d) > 2K_I$
%
% $$K_I > 0$

%% Problem 4b
Ki_new = 1;
J = 1;
B = 2;
G = tf( [ Kp_new, Ki_new],[J, (B+Kd_new), Kp_new,Ki_new]) 
figure(3)
rlocus(G)

%% Problem 4c
figure(4)
step(40*G)

%% Problem 4d

Kp4 = 1;
Kd4 = 1;

%%
% Using the Routh Hurwitz criteria, the following value is set for Kd.
% These values are then pluged in to the closed loop transfer funciton.
% This reduces the system to a 2nd order with one zero pole
Ki4 = 0;

G = tf( [ Kp4, Ki4],[J, (B+Kd4), Kp4,Ki4]) 
%%
% Ploting the root locus
figure(5)
rlocus(G)
figure(6)   
step(G)




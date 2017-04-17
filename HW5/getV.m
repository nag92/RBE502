function [ V ] = getV( q, qd )
%GETV Summary of this function goes here
%   Detailed explanation goes here
m1 = 10;
m2 = 5;
l1 = 1;
l2 = 1;
r1 = 0.5*l1;
r2 = 0.5*l2;
I1 = 10/12;
I2 = 5/12;
g = 9.81;
alpha = I1 + I2 + m1*r1^2 + m2*(l1^1 +l2^2);
beta =  m2*l2*r2;
gamma = I2 + m2*r2^2;

 V = [ -beta*sin(q(2))*qd(2)*qd(1) - beta*sin(q(2))*(qd(1)+qd(2))*qd(2) ; 
         beta*sin(q(2))*qd(1)*qd(2)];


end


function [ u ] = getTorque( q,qd,qdd )
%GETTORQUE Summary of this function goes here
%   Detailed explanation goes here
m1 = 10;
m2 = 5;
l1 = 1;
l2 = 1;
g = 9.81;
g = 9.81;


 
 M = getM(q);
 
 V = getV(q,qd);
 
 
 G =getG(q);
    
 
 %u = M*qdd + V + G;
 
end


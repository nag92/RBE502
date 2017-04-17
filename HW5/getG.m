function [ G ] = getG( q)
%GETG Summary of this function goes here
%   Detailed explanation goes here

m1 = 10;
m2 = 5;
l1 = 1;
l2 = 1;
g = 9.81;
 G = [ (m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2));...
     m2*g*l2*cos(q(1)+q(2))];


end


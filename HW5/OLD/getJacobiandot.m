function [ Jd ] = getJacobiandot( q1,q1d,q2,q2d )
%GETJACOBIANDOT Summary of this function goes here
%   Detailed explanation goes here
a1 = 0.3;
a2 = 0.3;

Jvd = [ -a1*q1d*cos(q1) + a2*(q1d+q2d)*cos(q1+q2), -a2*(q1d+q2d)*cos(q1+q2);...
        -a1*q1d*sin(q1) + -a2*(q1d+q2d)*cos(q1+q2), -a2*(q1d+q2d)*sin(q1+q2);...
        0,0];
    
Jwd = [ 0 ,0;...      
       0 , 0;...
      0, 0];
  
Jd= [Jvd];

end


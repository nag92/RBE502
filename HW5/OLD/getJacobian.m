function [ J ] = getJacobian( q1, q2 )
% getJacobian takes in joint angles and returns the jacobian 
%   
a1 = 0.3;
a2 = 0.3;

Jv = [ -a1*sin(q1) + a2*sin(q1+q2), -a2*sin(q1+q2);...
        a1*cos(q1) + a2*cos(q1+q2), a2*cos(q1+q2) ];
    
Jw = [ 0 ,0;...
      0 , 0;...
      1, 1];
  
J= [Jv];
       


end


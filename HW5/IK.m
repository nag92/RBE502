function [ theta1, theta2 ] = IK( x,y )
%IK finds IK of a 2 DOF arm 
%   
    a1 = 1;
    a2 = 1;
    theta2 = acos( (x^2 + y^2 - a1^2 - a2^2)/(2*a1*a2));
    theta1= atan2(y,x)- atan2( (a2*sin(theta2)),(a1 + a2*cos(theta2)));


end


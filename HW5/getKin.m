function [ q,qd,qdd ] = getKin( path,t )
%GETKIN get the kinmatics of a path at time t

q = path(1) + path(2)*t + path(3)*t^2 +path(4)*t^3;

qd = path(2) + 2*path(3)*t +3*path(4)*t^2;

qdd =  2*path(3) +6*path(4)*t;




end


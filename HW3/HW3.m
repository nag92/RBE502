%% Nathaniel Goldfarb
% HW3


%% Problem 1

%% Problem 2a

%% 
% $$ C = [ B, AB]$
A = [ 0 1;0 0];
B = [0;1];

C = [ B,A*B ]

if det(C) ~= 0
    disp('reachable')
end


%% Problem 2b

syms k1 k2 s

A_hat = A - B*[k1 k2]

controller = det( s*eye(2) - A_hat)

%%
% comparing equations we have
%
% $k1 = 1$
%
% $k2 = 0.14$

%% Problem 2c
%

t = 2;

time = [ 1 0 0 0;....
         0 1 0 0;...
         1 t t^2 t^3;...
         0 1 2*t 3*t^2];
vec = [-5;0;1;1];

a = time\vec

%% Problem 2d
%
%
% $$e = x - x_d$
%
% $$\dot{x} = Ax + Bu +L(y -Cx)$
% 


%% Problem 2e

e = det( s*eye(2) - A_hat)


%% Problem 3a


    

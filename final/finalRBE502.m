





%% Problem 3.a

syms x1 x2 x3 x4  a1 a2 u1 u2

f = [ x3;...
      x4;...
    -(3*x1*x2+x2^2);
    -x4*cos(x1)-3*(x1-x2)]  
  
 g = [0 0;...
      0 0;
      1 x2;...
      -3*x3*cos(x1)^2 1]

%% Problem 3.b  
  
f1 = -(3*x1*x2+x2^2) + u1 + x2*u2 - a1
f2 = -x4*cos(x1)-3*(x1-x2) -u1*3*x3*cos(x1)^2 + u2 - a2

 
F = [ f1==0, f2==0];
U = solve( F, [u1,u2]);

U.u1
U.u2

K = simplify( g*[U.u1;U.u2]+ f)
 
%% Problem 4.a
clear all
syms x1 x2 u

f = [  x2; -sin(x1)]
g = [0;1]

%% Problem 4.b

L = jacobian(f,[x1,x2])*g - jacobian(g,[x1,x2])*f 
%%
% Yes because this system can be driven to this point

%% Problem 4.c

A = [ 0 1;...
      0 0]
  
B = [0;1]

%%

[t,y] = ode45(@vdp1,[0 0],[2; 0]);

function dydt = vdp1(t,z)
    A = [ 0 1;...
      0 0];
  
    B = [0;1];
    K = [ 5; 5];
    v =  -( K*z' + sin(z(1)) );
    dydt = A*z + B*v

end






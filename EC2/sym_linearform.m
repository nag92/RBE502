%% Demostrate the robust control for 2dof PLANAR ARM: no gravity
% Reference:  Mark W Spong. On the robust control of robot manipulators. Automatic Control, IEEE Transactions on, 37(11):1782–1786, 1992.
% Implemented by : Jie Fu, jfu2@wpi.edu

% nominal parameters:
I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;

syms q1 q2 dq1 dq2 ddq1 ddq2
% The parameter of the system
% b1 = m1* r1^2 + m2*l1^2 + I1;
% b2 = m2*r2^2 + I2;
% b3 = m2*l1*r2;

syms b1 b2 b3
M = [(b1+b2)+2*b3*cos(q2), b2+b3*cos(q2);
    b2+b3*cos(q2), b2];
C = [-b3*sin(q2)*dq2, -b3*sin(q2)*(dq1+dq2); b3*sin(q2)*dq1,0];

dq =[dq1 ; dq2];
ddq= [ddq1; ddq2];

% We write M ddq  +  C dq  = Y(q,dotq, ddot q) b where b= [b1,b2,b3] are system parameters.

Y = equationsToMatrix(M*ddq + C*dq, [b1,b2,b3])

% define new state variables, 
% r  = dot e + Lambda*e;
% v = dot q_d - Lambda*e;
% a = dot v = ddot q_d - Lambda*dot e;
% e = q-q_d;

Lambda= sym('Lambda', [2,2]);
r = sym('r',[2,1]);
v = sym('v', [2,1]);
a = sym('a', [2,1]);
%ddq = dr + a;
%dq = r + v;

Y2 = equationsToMatrix(M*a + C*v, [b1,b2,b3])

%% Robust control implementation.
% tau = Y2(b0 +u) - K r;

    



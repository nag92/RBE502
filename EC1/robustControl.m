function []= robustControl(theta10,theta20,dtheta10, dtheta20,theta1f, theta2f,dtheta1f,dtheta2f,tf)
% Robust control design for 2-D planar arm.
% input: initial and final state.
% output: Demostrate the performance of robust controller with parameter
% uncertainty.
% the nominal model parameter:
m1 =10; m2=5; l1=1; l2=1; r1=0.5; r2 =.5; I1=10/12; I2=5/12; % parameters in the paper.
% the nominal parameter vector b0 is
b0 = [ m1* r1^2 + m2*l1^2 + I1; m2*r2^2 + I2; m2*l1*r2];

%% Trajectory planning block
% Initial condition
x0=[-0.5,0,-1,0.1];
x0e = [-0.7,0.5,-0.2,0]; % an error in the initial state.
xf=[0.8,0.5, 0, 0];
% The parameter for planned joint trajectory 1 and 2.
global a1 a2 % two polynomial trajectory for the robot joint
%%%%%%%
global torqueTime pholist etalist check % for keeping track of control inputs in ode45
%%%%%%%
nofigure=1;
a1 = planarArmTraj(theta10,dtheta10, theta1f, dtheta1f,tf, nofigure);
a2 = planarArmTraj(theta20,dtheta20, theta2f, dtheta2f,tf, nofigure);


torque=[];
%%%%%%%%%%%
torqueTime = [];
pholist = [];
etalist = [];
%%%%%%%%%%%
options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
%[T,X] = ode45(@(t,x)planarArmODEUncertain(t,x),[0 tf],x0e,options);
[T,X] = ode45(@(t,x)robust(t,x),[0 tf],x0e,options);
figure('Name','theta1');
plot(T, X(:,1),'r-');
hold on
plot(T, a1(1)+a1(2)*T+ a1(3)*T.^2+a1(4)*T.^3,'b-');
title('Theta_1 under Robust Control');
figure('Name','theta2');
plot(T, X(:,2),'r-');
hold on
plot(T, a2(1)+a2(2)*T+ a2(3)*T.^2+a2(4)*T.^3, 'b-');
title('Theta_2 under Robust Control');
%%%%%%%%
uIdx = resampleTime(T,torqueTime);
figure
plot(T,torque(1,uIdx))
hold on
plot(T,torque(2,uIdx))
xlabel seconds
xlabel N-m
title 'Input Torque'
legend('\tau_1','\tau_1')
%%%%%%%%
figure(4)
hold on;
plot(pholist)
plot(etalist)
legend('pho','eta')

figure(5)
plot(check)
mean(check)
function [dx] = robust(t,x)
     K= 10*eye(2);
        Lambda= 5*eye(2);
        % Compute the desired state and their time derivatives from planned
        % trajectory.
        vec_t = [1; t; t^2; t^3]; % cubic polynomials
        theta_d= [a1'*vec_t; a2'*vec_t];
        %ref = [ref,theta_d];
        % compute the velocity and acceleration in both theta 1 and theta2.
        a1_vel = [a1(2), 2*a1(3), 3*a1(4), 0];
        a1_acc = [2*a1(3), 6*a1(4),0,0 ];
        a2_vel = [a2(2), 2*a2(3), 3*a2(4), 0];
        a2_acc = [2*a2(3), 6*a2(4),0,0 ];
        dtheta_d =[a1_vel*vec_t; a2_vel* vec_t];
        ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t];
        theta= x(1:2,1);
        dtheta= x(3:4,1);
        
        %error
        epsilon_m2 = 1;
        epsilon_r2 = 0.5;
        epsilon_I2 = (15/12); 
        
        %the true model
        m2t = m2 + epsilon_m2*rand(1);% m1 true value is in [m1, m1+epsilon_m1] and epsilon_m1 a random number in [0,10];
        r2t = r2 + epsilon_r2*rand(1);
        I2t = I2 + epsilon_I2*rand(1);
        
        a = I1+I2+m1*r1^2+ m2t*(l1^2+ r2t^2);
        b = m2t*l1*r2t;
        d = I2t+ m2t*r2t^2;
        
        % the actual dynamic model of the system is characterized by M and
        % C
        Mmat = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        Cmat = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];
        invM = inv(Mmat);
        invMC = invM*Cmat;
        
        %upper
        m2t = m2 + epsilon_m2;% m1 true value is in [m1, m1+epsilon_m1] and epsilon_m1 a random number in [0,10];
        r2t = r2 + epsilon_r2;
        I2t = I2 + epsilon_I2;
        a = I1+I2+m1*r1^2+ m2t*(l1^2+ r2t^2);
        b = m2t*l1*r2t;
        d = I2t+ m2t*r2t^2;
        M_upper = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        C_upper = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];


        %lower
        m2t = m2 ;% m1 true value is in [m1, m1+epsilon_m1] and epsilon_m1 a random number in [0,10];
        r2t = r2 ;
        I2t = I2 ;
        a = I1+I2+m1*r1^2+ m2t*(l1^2+ r2t^2);
        b = m2t*l1*r2t;
        d = I2t+ m2t*r2t^2;
        M_lower = [a+2*b*cos(x(2)), d+b*cos(x(2));  d+b*cos(x(2)), d];
        C_lower = [-b*sin(x(2))*x(4), -b*sin(x(2))*(x(3)+x(4)); b*sin(x(2))*x(3),0];

        % get other Ms
        M_hat = M_lower;%2*inv(M_lower+M_upper)*eye(2);
        C_hat = C_lower;2*inv(C_lower+C_upper)*eye(2);
        M_squiggy = Mmat - M_hat;
        C_squiggy = Cmat - C_hat;
        % error
        e1 = theta-theta_d;
        e2 = dtheta-dtheta_d;
        
        e = [e1; e2];
        
        % Random coef
       
        g1 = 15;
        g2 = 10;
        g3 = 8;
        K0 = [ 150 0; 0 90 ];
        K1 = [75 0; 0  10];
        
        A = [ 0 0 1  0;
              0 0 0  1;  
               -K0 -K1];
        B = [0,0;0 0;eye(2)];
        Q = [ 100 0 0 0;
              0  100 0 0;
              0 0 100 0;
              0 0 0 100];
        P = lyap(A,Q) ;
        
        normE = norm(e);
        pho = ( g1*normE + g2*normE^2 + g3);
        pholist = [pholist,pho];
        % get the linear error system
        
        % get delta_alpha
       
      
        temp = transpose( B )*P*e;
        check = [check, norm(temp)];
        %controller 1
%         if norm(temp) ==0
%             delta_alpha = 0;
%         else
%             delta_alpha = -pho*(temp/norm(temp));
%         end
        %
       
     
        ep =  470.8419;
        if norm(temp)  >  ep
            delta_alpha = -pho*(temp/norm(temp));;
        else
            delta_alpha = -pho*(temp)/ep;
        end
        
        
        a_q = theta_d - K0*e1 - K1*e2 + delta_alpha;
        eta = invM*( M_squiggy*a_q +  C_squiggy*dtheta  );
        etalist = [etalist,norm(eta)];
        
        qdd = a_q + eta;
        
        tau = Mmat*qdd+Cmat*dtheta;
        torque =[torque, tau];
        %%%%%%%%
        torqueTime =[torqueTime, t];
        %%%%%%%%
        dx=zeros(4,1);
        dx(1) = x(3);
        dx(2) = x(4);
        dx(3:4) = -invMC* x(3:4) +invM*tau;
        
end
end
%%




%%
function uIdx = resampleTime(T,inputTime)
% Input:
%   T - ode45 output time values
%   inputTime - time's recorded within ode45
% This first loop filters out torque indexes that are repeated as
% ode45 restarts iterations.
uIdx = zeros(length(T),1);
for i = 1:length(T)
    try
        uIdx(i) = find(T(i)==inputTime,1,'last');
    catch ME
        if strcmp(ME.identifier,'MATLAB:subsassignnumelmismatch')
            if i>1
                uIdx(i) = uIdx(i-1);
            else
                uIdx(i) = 1;
            end
        else
            rethrow ME
        end
    end
end
end

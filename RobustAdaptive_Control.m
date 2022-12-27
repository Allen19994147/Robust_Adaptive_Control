clc;clear
clear figure
%%% Robust Adaptive Control simulation
%%% Assume that the values of the system are const but unknown

%%% Parameters define
%%% Dynamics: J*x_doubledot + b*x_dot*abs(x_dot) + mgl*sin(x) = u;
J = 10.98;      %%%
b = 23.542;
mgl = 50.53;
P = diag([1 1 0.5]); %%% should be const diagonal matrix
P = inv(P);
Kd = 4;     %%% should be greater than 0
lemmda = 1; %%% should be greater than 0
a = [b mgl J]';
a_hat = [10 10 10]';    %Pure initial guess

phi = 0.005;
Time = 50;          %Total simulation time
interval = 0.005;   % Time interval

%%% Desired Trajectory and IC
xd = pi;    
xd_dot = 0;
xd_doubledot = 0;
x = 0;
x_dot = 0;
x_double_dot = 0;
X = [];
A = [];
S = [];
for i = 0:interval:Time
    xd = 1.7*i + 10 ;   %%Position Command
    xd_dot =  1.7 ;     %%Velocity Command
    %%%
    s = (x_dot - xd_dot) + lemmda*(x-xd);
    s_delta = s - phi*sat(s/phi);
    xr_doubledot = xd_doubledot - lemmda*(x_dot-xd_dot);
    Y = 1.*[x_dot*abs(x_dot) sin(x) xr_doubledot];
    a_hat_dot = (-s_delta.*Y*P)';
    a_hat = a_hat + interval.*a_hat_dot;
    u = Y*a_hat - Kd*s_delta;     %%% Control u here
    
    x_double_dot = (u - b*x_dot*abs(x_dot) - mgl*sin(x))/J; %Real state update
    x_dot = x_dot + interval*x_double_dot;
    x = x + interval*x_dot;
%     while(abs(x) > pi)
%         if(x>0)
%             x = x - 2*pi;
%         else
%             x = x + 2*pi;
%         end
%     end
    X = [X [x;xd;x_dot;xd_dot]];
    A = [A a_hat];
    S = [S [s;s_delta]];
end

%%%Visualization
X;
A;
figure(1)
hold on
title("Commands Tracking")
plot(X(1,:),'-','color','red')
plot(X(2,:),'--','color','green')
plot(X(3,:),'-','color','blue')
plot(X(4,:),'--','color','black')
legend('Posi','Posi Commamd','Velo','Velo Command')

hold off
figure(2)
hold on
title("Parameter Estimations")
plot(A(1,:))
plot(A(2,:))
plot(A(3,:))
legend('J','b','mgl')

hold off
figure(3)
hold on
title("Tracking Errors")
plot(X(1,:)-X(2,:),'-','color','yellow')
plot(X(3,:)-X(4,:),'-','color','blue')
legend('Posi','Velo')
hold off

figure(4) 
hold on
title("Parameter Estimation Errors")
plot(J-A(1,:),'--','color','red')
plot(b-A(2,:),'--','color','black')
plot(mgl-A(3,:),'--','color','blue')
legend('J err','b err','mgl err')
hold off
function satvalue = sat(m)
if (abs(m) > 1)
    satvalue =  abs(m)/m;
else
    satvalue = m;
end
end

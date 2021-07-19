% Kalman Filter HW2
clear;
T = 100; 
wn = 2; 
zeta = 0.7; 
b = -2 * zeta * wn;

A = [0 1; -wn^2 b];
B = [1;1/sqrt(2)];
Qk = [0.01 0.001; 0.001 0.02]; 
Rk = (1/2)^2;
Ck = [1 0]; % measurement matrix

[PHI GAM] = c2d(A, B, 1);


x = [0; 0]; % initial state
xhat = [0;0];
xbar = [0;0];
%Lk = [0;0];
Sigma = [0 0; 0 0];
Sigbar = [0 0; 0 0];
zk = 0;
% Initialize arrays for later plotting
xArray = x;
xhatArray = xhat;
zArray = zk;

xhat2 = [0;0];
xhatArray2 = xhat2;
[pp] = dare(PHI', Ck', Qk, Rk);
Linf = pp*Ck'*inv(Ck*pp*Ck'+ Rk);


xhat3 = [0;0];
xhatArray3 = xhat3;
[Y2, Linf2, eig_se2] = idare(PHI', Ck', Qk, Rk);
%Linf2 = Linf2';
%Linf2 = inv(PHI)*Linf2;


for t = 1 : T 
    % Simulate the system.
    %w = sqrt(Q(1,1)) * randn;
    Uk = sin(0.1*(t-1));
    %Uk = 0;
       
    Wk = chol(Qk, 'lower')*randn(2, 1);
    Vk = chol(Rk, 'lower')*randn;
    
    F = expm(A);    
    x = F*x + Wk + B*Uk;
    zk = Ck * x + Vk;
    
    % Kalman filter
    % Prediction
    xbar = F*xhat + B*Uk;
    Sigbar = F*Sigma*F' + Qk;
    
    % Correction
    Lk = Sigbar*Ck'*inv(Ck*Sigbar*Ck' + Rk);
    xhat = xbar + Lk*(zk - Ck*xbar);
    %Sigma = (eye(2)-Lk*Ck)*Sigbar;    
    Sigma = (eye(2)-Lk*Ck)*Sigbar*(eye(2)-Lk*Ck)'+Lk*Rk*Lk';
    
    
    xhat2 = xbar + Linf*(zk - Ck*xbar);
    xhatArray2 = [xhatArray2 xhat2];
    
    xhat3 = xbar + Linf*(zk - Ck*xbar);
    xhatArray3 = [xhatArray3 xhat3];
    
    
    xArray = [xArray x];
    xhatArray = [xhatArray xhat];
    zArray = [zArray zk];
end

% Plot results
%close all
t = 0 : T;

figure(1);
plot(t, xArray(1,:), 'r-', t, xhatArray(1,:), 'b-*', t, zArray(1,:), 'g+');

set(gca,'FontSize',12); set(gcf,'Color','White');
ylabel('x');
xlabel('time');
legend('true state', 'estimated state', 'Meas.');

figure(2);
plot(t, xArray(1,:), 'r-', t, xhatArray2(1,:), 'b-*', t, zArray, 'g+');
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('time');
legend('true state', 'estimated state', 'Meas.');

figure(3);
plot(t, xArray(1,:), 'r-', t, xhatArray3(1,:), 'b-*', t, zArray, 'g+');
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('time');
legend('true state', 'estimated state', 'Meas.');
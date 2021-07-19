% Kalman Filter HW 1
clear;

T = 100; 

xtrue = 0; 
xhat = 0; 
xbar = 0;
Sigma = 0;
Sigbar = 0;

xhat2 = 0;
xbar2 = 0;
Sigma2 = 0;
Sigbar2 = 0;



xArray = [];
xhatArray = [];
xhatArray2 = [];
zArray = [];
LkArray = [];

% System
A = 0.7;
B = 1/sqrt(2);
Uk = 10;
Ck = 1;
Qk = 0.5;
Rk = 0.15;


% method 1
[pp] = dare(A', Ck', Qk, Rk);
Linf = pp*Ck'*(Ck*pp*Ck'+ Rk)^(-1);


[Y2, Linf2, eig_se2] = idare(A', Ck', Qk, Rk);
Linf2 = Linf2';
Linf2 = inv(A)*Linf2;

for t = 0 : T
   % True Value
   Wk = sqrt(Qk)+randn;
   Vk = sqrt(Rk)+randn;
   
   xtrue = A*xtrue + B*Uk + Wk;
   zk = Ck*xtrue +Vk;
   
   % Kalman filter
   % Prediction
   xbar = A*xhat + B*Uk;
   Sigbar = A*Sigma*A + Qk;
   
   % Correction
   Lk = Sigbar*Ck/(Ck*Sigbar*Ck + Rk);
   xhat = xbar + Lk*(zk - Ck*xbar);
   %Sigma = (1-Lk*Ck)*Sigbar*(1-Lk*Ck)'+Lk*Rk*Lk';
   Sigma = (1-Lk*Ck)*Sigbar;  
   
   
   xhat2 = xbar + Linf2*(zk - Ck*xbar);
    
   xArray = [xArray; xtrue];
   xhatArray = [xhatArray; xhat];
   xhatArray2 = [xhatArray2; xhat2];
   zArray = [zArray; zk];
   LkArray = [LkArray; Lk];
end

% Plot results
%close all;
t = 0 : T;

figure(1);
plot(t, xArray, 'r-', t, xhatArray, 'b-*', t, zArray, 'g+');
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('time');
legend('true state', 'estimated state', 'Meas.');

figure(2);
plot(t, xhatArray, 'r-', t, xhatArray2, 'b-*');


figure(3);
plot(t, xArray, 'r-', t, xhatArray2, 'b-*', t, zArray, 'g+');
set(gca,'FontSize',12); set(gcf,'Color','White');
xlabel('time');
legend('true state', 'estimated state', 'Meas.');
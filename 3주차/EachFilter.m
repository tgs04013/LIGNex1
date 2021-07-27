clc


qx = 3;
Ts = 0.1;
Tf = 500;
x_noise = [];
y_noise = [];

A_cv = [0 1 0 0 ; 0 0 0 0 ; 0 0 0 1 ; 0 0 0 0 ];

Ck_cv = [ 1 0 0 0 ; 0 0 1 0 ];


x0 = [0; 200; 0;  200];

P0 = diag([100^2, 10^2, 100^2, 10^2]);


RK = [ 1 0 ; 1 0 ];

xhat_cv(:,1) = x0;
xbar_cv(:,1) = xhat_cv(:,1);



Sigma_cv = P0;
Sigbar_cv = P0;



t = 0 : Ts : Tf;
sample_size = size(x);

qx = qx*square(Ts); 
qy = qx;

%Qk_cv = [  Ts^3*qx^2/3  Ts^2*qx^2/2 0               0 ;
% %          Ts^2*qx^2/2  Ts*qx*qx    0               0 ;
%           0            0           Ts^3*qy^2/3     Ts^2*qy^2/2 ;
%           0            0           Ts^2*qy^2/2     Ts*qy^2 ];

Qk_cv = P0;
Qk_cv = 12*Qk_cv;    
%F_cv = expm(A_cv*Ts);
F_cv = A_cv;


Rk = [ 1 0;
    0 1];

for i = 1:sample_size(2)-1
    
    % Prediction
    xbar_cv(:,i+1) = F_cv*xhat_cv(:,i);
    Sigbar_cv = F_cv*Sigma_cv*F_cv' + Qk_cv;
    
    x_seeker = [ x_seeker x(i) + rand()];
    y_seeker = [ y_seeker y(i) + rand()];
    
    % Correction
    Lk_cv = Sigbar_cv*Ck_cv'*inv(Ck_cv*Sigbar_cv*Ck_cv' + Rk);
    xhat_cv(:,i+1) = xbar_cv(:,i+1) + Lk_cv*([x(i);y(i)] - Ck_cv*xbar_cv(:,i+1));
    Sigma_cv = (eye(4)-Lk_cv*Ck_cv)*Sigbar_cv;
    
    
end

figure(1)

plot(x, y)

hold on
plot(xhat_cv(1,:), xhat_cv(3,:));
hold off


figure(2)
plot(x_seeker, y_seeker)
hold on
plot(xhat_cv(1,:), xhat_cv(3,:));
hold off



clc
clear all
global Vm x  y x_target y_target
UU = [];
x = [];
y = [];

A = [ 0 1;  0 0 ];
U = 0;
x0 = 0;  % 미사일 초기 x 위치
y0 = 0;   % 미사일 초기 y 위치
x_target = 1000;   % 타켓 x 위치
y_target = 0;   % 타켓 y 위치
 
x2 = [];    
y2 = [];
 

Vm = 200;   % 미사일 고정 속도
R = sqrt(( y_target - y0 )^2 + ( x_target - x0 )^2);    % 미사일과 타켓 사이의 거리 r 
 
lamda = atan2(y_target - y0, x_target - x0);    % 미사일과 타켓 사이의 각 lambda radian 값
yaw = (10+lamda*180/pi)* (pi/180);  % 미사일의 비행경로각 gamma radian 값
%Mtheta = 5* (pi/180);
yaw*180/pi      
lamda*180/pi
(yaw-lamda)*180/pi 
 
X(:,1) = [R;yaw-lamda]; 
Tf=500; % final time
Ti=0.01;    % 시간 간격 0.01s
t=0:Ti:Tf;  % 0 ~ 500 초 사이 시간 
sample_size = size(t,2);
 
n = 3;

xhat = 0;
C = [1 0];

qx = 3;

Tf = 500;
x_noise = [];
y_noise = [];

A_cv = [0 1 0 0 ; 0 0 0 0 ; 0 0 0 1 ; 0 0 0 0 ];

Ck_cv = [ 1 0 0 0 ; 0 0 1 0 ];


x0 = [0; 0; 0;  0];

P0 = diag([100^2, 10^2, 100^2, 10^2]);


RK = [ 1 0 ; 1 0 ];

xhat_cv(:,1) = x0;
xbar_cv(:,1) = xhat_cv(:,1);



Sigma_cv = P0;
Sigbar_cv = P0;



sample_size = size(x);

qx = qx; 
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
    
    X(:,i+1) = rk(X(:,i), U,Ti); 

    if X(1,i) < 5    
        break
    end
    
    % Prediction
    xbar_cv(:,i+1) = F_cv*xhat_cv(:,i);
    Sigbar_cv = F_cv*Sigma_cv*F_cv' + Qk_cv;
    
    U = n*Vm*(X(2,i+1)-X(2,i)) ;    % a = nV\dot{sigma}
    UU = [UU U ];     % 입력 UU 에 저장
    
    x = [x x0+(R-X(1,i+1))*cos(lamda+X(2,i+1))];    
    y = [y y0+(R-X(1,i+1))*sin(lamda+X(2,i+1))];
    x_seeker = x + randn;
    y_seeker = y + randn;
        
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

plot(x, y)


function dx=plant(x,u)
    % x(1) = r , x(2) = sigma
    global Vm
    dx(1,1) = -Vm*cos(x(2));   % 거리 변화율 \dot{r}
    dx(2,1) = -Vm*sin(x(2))/x(1) + u/Vm;    % 각도 변화율 \dot{sigma}
    
end
 
% Runge-Kutta method 수치 해석
function dx=rk(x,u,T)
    k1=plant(x,u)*T;    
    k2=plant(x+k1*0.5,u)*T;
    k3=plant(x+k2*0.5,u)*T;
    k4=plant(x+k3,u)*T;
    dx=x +((k1+k4)/6+(k2+k3)/3);   
end


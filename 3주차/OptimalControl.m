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
x_target = 10000;   % 타켓 x 위치
y_target = 400;   % 타켓 y 위치
 
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

for i=1:sample_size-1        
    X(:,i+1) = rk(X(:,i), U,Ti); 
    U = n*Vm*(X(2,i+1)-X(2,i)) ;    % a = nV\dot{sigma}
    if X(1,i) < 5    
        break
    end
    
%     X(1,i+1) = X(1,i+1) + 2*rand();


    UU = [UU U ];     % 입력 UU 에 저장
    x = [x x0+(R-X(1,i+1))*cos(lamda+X(2,i+1))];    
    y = [y y0+(R-X(1,i+1))*sin(lamda+X(2,i+1))];        
    
end

t2 = 0:Ti:Ti*i;
tf = Ti * i
 
figure(1)   % 위치 (x , y) 좌표
plot(x,y)   
 
figure(2)   % 거리 r 변화 그래프     
plot(t2,X(1,:))
 
figure(3)   % sigma 각도 변화 그래프
plot(t2,(X(2,:))*180/pi)
 
figure(4)   % 입력 a 의 값
plot(UU)
 
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

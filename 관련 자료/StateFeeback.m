clear all

% motor 파라메타
R = 2.06; Kt = 0.0235; Kb = 0.0235;
J = 1.07e-6; L = 2.38e-6; Beta = 3.5077e-6;

%상태 방정식 정의
A = [ 0 1 0 ; 0 -Beta/J Kt/J ; 0 -Kb/L -R/L ];
B = [ 0 0 1/L]';
C = [ 1 0 0 ];

Ti = 0.05;

%디스크리트 설계
F = expm(A*Ti);

syms z
G = int(expm(A*z)*B, 0, Ti);
G = double(G);

%StateFeeback Factor 선정
p = [0.2 0.2+0.3*i 0.2-0.3*i];
Kd = place(F,G,p);

UU = [];
X(:,1) = [90;0;0];
Xhat(:,1) = [0;0;0];

Tf = 3;
t = 0 : Ti : Tf;
sample_size = size(t,2);
ref = 10;

for i = 1:sample_size-1
    U = -Kd*X(:,i);
    X(:, i+1) = F*X(:,i)+G*U ;
    UU = [ UU U ];
end

figure(1)   
stem(t,X(1,:))
figure(2)  
stem(UU)
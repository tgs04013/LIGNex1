clear all
A = [ 0 1 ; 0 0];
B = [ 0 1 ]'
C = [1 0];

Ti = 0.1;

%디스크리트 설계
F = expm(A*Ti);

syms z
G = int(expm(A*z)*B, 0, Ti);
G = double(G);
%G = subs(G, 'T', Ti)

syms s k1 k2
det(s*eye(2)-A+B*[k1 k2])
%det(s*eye(2)-F-G*[k1 k2])
p = roots([1 1 0.5] )
%K = place(A,B,p);
K = place(F,G,exp(p'*Ti));
%Kd = place(F,G,exp(p'*Ti));

UU = [];
X(:,1) = [5;0];
Xhat(:,1) = [0;0];

Tf = 20;
t = 0 : Ti : Tf;
sample_size = size(t,2);

for i = 1:sample_size-1
    e = C*X(:,i) - C*Xhat(:,i);
    U = -K*X(:,i);    
    X(:, i+1) = F*X(:,i)+G*U;
    Xhat(:,i+1) = F*Xhat(:,i)+G*U*e;
    UU = [ UU U ];
end

figure(1)
stem(t,X(1,:))
hold on
plot(t,X(2,:))
hold off
figure(2)
plot(UU)
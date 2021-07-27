syms X1 X2 U

Vm = 200
 
fx=[ -Vm*cos(X2);
    -Vm*sin(X2)/X1 + U/Vm ]
 
sys = jacobian(fx,[X1 X2 U])
subs(sys, [X1 X2 U], [ Vm pi/2 Vm ])
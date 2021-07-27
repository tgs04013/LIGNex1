function dx = mass_spring_damper_diff(t,x)

k = 1;
m = 5;
c = 1;
F = 1;

dx = zeros(2,1); 

dx(1) = x(2);
dx(2) = -(k/m)*x(1)-(c/m)*x(2)+(1/m)*F;




clear 
clc

tspan = [0 50];
x0 = [0;0];

[t,x] = ode23(@mass_spring_damper_diff, tspan, x0);

plot(t,x)
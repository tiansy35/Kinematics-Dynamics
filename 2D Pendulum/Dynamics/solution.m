clc; clear all;

% Calculate ODE
tspan = [0,10];
y0 = [0; 0; 0; 0];
[t,y] = ode45('odefun',tspan,y0);

% Plot solution for joint variables
figure(1);
size = size(t);
plot(t,y(:,1),t,y(:,3))
xlabel('Time /s')
ylabel('Angle /rad')
legend('theta1','theta2')

% Plot solution for energy
for i = 1:size(1)
    Kt(i,1) = y(i,2)*y(i,4) + (3*y(i,2)^2)/2 + y(i,4)^2/2 + y(i,2)^2*cos(y(i,3)) + y(i,2)*y(i,4)*cos(y(i,3));
    Pt(i,1) = 10*sin(y(i,1) + y(i,3)) + 20*sin(y(i,1));
end
Etot = Kt+Pt;
figure(2);
plot(t,Kt,t,Pt,t,Etot)
xlabel('Time /s')
ylabel('Energy /J')
axis([-inf,inf,-35,40]);
legend('Kinetic Energy','Potential Energy','Total Energy')



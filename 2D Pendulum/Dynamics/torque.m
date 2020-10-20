% Plot torque

% Get symbolic representation of tau
tau = dynamics();
load('symbol');

% Plug in numerical values
tau = subs(tau,g,10);
tau = subs(tau,{m1,m2},{1,1});
tau = subs(tau,{L1,L2},{1,1});
ans = tau;
syms t

tau = subs(tau,{t1,t2,dt1,dt2,ddt1,ddt2},{t,t,1,1,0,0})
T = [0:0.01:10];
y1 = subs(tau(1),t,T);
y2 = subs(tau(2),t,T);

plot(T,y1,T,y2)
xlabel('Time /s')
ylabel('Torque /N*m')
legend('1st Joint','2nd Joint')


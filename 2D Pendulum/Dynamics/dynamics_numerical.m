% Get symbolic representation of tau
tau = dynamics();
load('symbol');

% Plug in numerical values
tau = subs(tau,g,10);
tau = subs(tau,{m1,m2},{1,1});
tau = subs(tau,{L1,L2},{1,1});

K = subs(K,g,10);
K = subs(K,{m1,m2},{1,1});
K = subs(K,{L1,L2},{1,1});

P = subs(P,g,10);
P = subs(P,{m1,m2},{1,1});
P = subs(P,{L1,L2},{1,1});

% Solve expression for thetadotdot (rad/(s^2))
eq1 = tau(1)==0;
eq2 = tau(2)==0;
[x,y] = solve([eq1,eq2],[ddt1,ddt2]);

% Replace the names of variables to fit odefunction
x = char(x);
x = strrep(x,'dt1','y(2)');
x = strrep(x,'dt2','y(4)');
x = strrep(x,'t1','y(1)');
x = strrep(x,'t2','y(3)')
y = char(y);
y = strrep(y,'dt1','y(2)');
y = strrep(y,'dt2','y(4)');
y = strrep(y,'t1','y(1)');
y = strrep(y,'t2','y(3)')

% Modify K and P to plot energy
K = char(K);
K = strrep(K,'dt1','y(i,2)');
K = strrep(K,'dt2','y(i,4)');
K = strrep(K,'t1','y(i,1)');
K = strrep(K,'t2','y(i,3)')

P = char(P);
P = strrep(P,'dt1','y(i,2)');
P = strrep(P,'dt2','y(i,4)');
P = strrep(P,'t1','y(i,1)');
P = strrep(P,'t2','y(i,3)')









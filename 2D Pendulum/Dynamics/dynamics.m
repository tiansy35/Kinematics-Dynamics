function tau = dynamics()
% Derive the symbolic representation
% for the Euler-Lagrange equation
%
%
%%
syms q pos L m dq
syms t1 t2
syms dt1 dt2
syms L1 L2 m1 m2
syms g

n = 2;
q = [t1 t2];
dq = [dt1 dt2];
L = [L1 L2];
m = [m1 m2];
pos = [0;0];
theta = 0;

% Calculate joint positions
for i = 1:n
    theta = theta+q(i);
    posi = [L(i)*cos(theta); L(i)*sin(theta)];
    poscurr = pos(:,i);
    pos = [pos poscurr+posi];
end

% Calculate velocity on joints
syms V
V = sym(zeros(3,n));
z = [0; 0; 1];
for i = 1:n
    for j = 1:i
        Ovec = [pos(:,i+1);0]-[pos(:,j);0];
        Vj = dq(j)*cross(z,Ovec);
        V(:,i) = V(:,i)+Vj;
    end   
end
V = V(1:2,:);

% Calculate kinetic energy
syms K
K = 0;
for i = 1:n
    Ki = (1/2)*m(i)*(V(1,i)^2+V(2,i)^2);
    K = K+Ki;
end
K = simplify(K);

% Calculate potential energy
syms P
P = 0;
for i = 1:n
    Pi = m(i)*g*pos(2,i+1);
    P = P+Pi;    
end
P = simplify(P);

La = K-P; % Lagrangian

% Euler-Lagrange equation
partial_q = sym(zeros(n,1));
partial_dq = sym(zeros(n,1));
for i = 1:n
    partial_q(i) = diff(La,q(i));
    partial_dq(i) = diff(La,dq(i));
end

syms t
syms T1(t) T2(t) % Set joint variables as functions of t
syms tau ddt1 ddt2

partial_q = subs(partial_q,{t1,t2,dt1,dt2},{T1(t),T2(t),diff(T1(t)),diff(T2(t))});
partial_dq = subs(partial_dq,{t1,t2,dt1,dt2},{T1(t),T2(t),diff(T1(t)),diff(T2(t))});
tau = diff(partial_dq,t)-partial_q;   % Generalized force
tau = subs(tau,{T1(t),T2(t),diff(T1(t),t),diff(T2(t),t),diff(T1(t),t,t),diff(T2(t),t,t)},{t1,t2,dt1,dt2,ddt1,ddt2});

save('symbol');

end


function [Q] = InverseK(T0e)
%Calculate the inverse kinematics of the lynx manipulator
% 
L1 = 76.20; L2 = 146.05; L3 = 187.325; L4 = 34; L5 = 42.2; Le = 28.575; %(unit:mm)
Pe = T0e(1:3,4); %position of the end effector
d6 = L4+L5+Le;
r3 = T0e(1:3,3);
Pc = Pe-d6*r3; %position of the wrist center
x = Pc(1); y = Pc(2); z = Pc(3);

t1(1) = atan2(y,x);
if t1(1) > 0
    t1(2) = t1(1)-pi;
else t1(2) = t1(1)+pi;  %two solutions of theta1
end
alpha = atan2(sqrt(x^2+y^2),L1-z);
Cbeta = (L2^2+x^2+y^2+(z-L1)^2-L3^2)/(2*L2*sqrt(x^2+y^2+(z-L1)^2));
beta = atan2(sqrt(1-Cbeta^2),Cbeta);
t2(1) = pi-alpha-beta;        %four solution of t2, t2(1) and t2(2) correspond to t1(1), t2(3) and t2(4) correspond to t1(2)
t2(2) = pi-(alpha-beta);       
t2(3) = -t2(1);                
t2(4) = -t2(2);                 
Ctheta = (L2^2+L3^2-(x^2+y^2+(z-L1)^2))/(2*L2*L3);
t3(1) = pi/2-atan2(sqrt(1-Ctheta^2),Ctheta); %two solution of t3
t3(2) = pi/2+atan2(sqrt(1-Ctheta^2),Ctheta);
t3(3) = t3(2);
t3(4) = t3(1);
for i = 1:4
    if t3(i) > pi
        t3(i) = t3(i)-2*pi;
    end
    if t3(i) < -pi
        t3(i) = t3(i)+2*pi;
    end
end

T03 = [];
T06 = [];
R = T0e(1:3,1:3);
for i = 1:4
    if i < 3
        A1 = DH(0,-pi/2,L1,t1(1));
    else  A1 = DH(0,-pi/2,L1,t1(2));
    end
    A2 = DH(L2,0,0,-pi/2+t2(i));
    A3 = DH(L3,0,0,pi/2+t3(i));
    T03 = A1*A2*A3; 
    R03 = T03(1:3,1:3);           %calculate R03
    R35 = (R03')*R;               %calculate R35
    sin4 = R35(2,3); cos4 = R35(1,3);       %calculate theta4 and theta5
    t4(i) = atan2(sin4,cos4);
    sin5 = -R35(3,1); cos5 = -R35(3,2);
    t5(i) = atan2(sin5,cos5);  
end

Q1 = [t1(1); t1(1); t1(2); t1(2)];
Q = [Q1,t2.',t3.',t4.',t5.'];






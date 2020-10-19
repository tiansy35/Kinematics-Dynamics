function [jointPositions,T0e] = calculateFK(q)
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

% Compute intermediate transformation matrices
%syms l1 l2 l3 l4 l5 t1 t2 t3 t4 t5;
L1 = 76.20; L2 = 146.05; L3 = 187.325; L4 = 34; L5 = 42.2; Le = 28.575; %(unit:mm)
t1=q(1);t2=q(2);t3=q(3);t4=q(4);t5=q(5);
A1=DH(0,-pi/2,L1,t1);
A2=DH(L2,0,0,-pi/2+t2);
A3=DH(L3,0,0,pi/2+t3);
A4=DH(0,-pi/2,0,t4-pi/2);
A5=DH(0,0,L4+L5,t5);
A44=DH(L4,0,0,t4);
Ae=DH(0,0,Le,0);

% Compute transformation matrices to each joint
T1=A1;
T2=A1*A2;
T3=A1*A2*A3;
T4=A1*A2*A3*A4;
T5=A1*A2*A3*A4*A5;
T44=A1*A2*A3*A44;
T0e=T5*Ae;

% Compute joint positions
p0=zeros(3,1);
p1=T1(1:3,4);
p2=T2(1:3,4);
p3=T3(1:3,4);
p4=T44(1:3,4);
p5=T5(1:3,4);
jointPositions=[p0,p1,p2,p3,p4,p5].';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end



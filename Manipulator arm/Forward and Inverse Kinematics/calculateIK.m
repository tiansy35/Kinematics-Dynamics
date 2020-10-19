function [q isPos] = calculateIK(T0e)
% INPUT:
%   T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)
%
% OUTPUT:
%   q          - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
%                which are required for the Lynx robot to reach the given 
%                transformation matrix T. Each row represents a single
%                solution to the IK problem. If the transform is
%                infeasible, q should be all zeros.
%   isPos      - a boolean set to true if the provided
%                transformation T is achievable by the Lynx robot, ignoring
%                joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

L1 = 76.20; L2 = 146.05; L3 = 187.325; L4 = 34; L5 = 42.2; Le = 28.575; %(unit:mm)
Pe = T0e(1:3,4); %position of the end effector
d6 = L4+L5+Le;
r1 = T0e(1:3,1);
r2 = T0e(1:3,2);
r3 = T0e(1:3,3);
Pc = Pe-d6*r3; %position of the wrist center
x = Pc(1); y = Pc(2); z = Pc(3);

max = L2+L3+L4+L5+Le; 
P1 = [0 0 L1];
a = Pe(1)-P1(1);
b = Pe(2)-P1(2);
c = Pe(3)-P1(3);
d = a^2+b^2+c^2;
Q = [];
if d > max^2   %determine whether the end effector position is outside the workspace
    isPos = 0;
    q = Q;
else if (x == 0) && (y == 0) && (Pe(1) == 0) && (Pe(2) == 0) %special situation when q1 and q5 have infinite solutions 
        if z > 0
            d0 = z-L1;
            cos2 = (L2^2+d0^2-L3^2)/(2*L2*d0);
            cos3 = (L2^2+L3^2-d0^2)/(2*L2*L3);
            cos4 = (L3^2+d0^2-L2^2)/(2*L3*d0);
            Q(2) = atan2(sqrt(1-cos2^2),cos2);               %calculate joint variables for this special situation
            Q(3) = -3*pi/2+atan2(sqrt(1-cos3^2),cos3);
            Q(4) = atan2(sqrt(1-cos4^2),cos4);
        else
            d0 = -z+L1;
            cos2 = (L2^2+d0^2-L3^2)/(2*L2*d0);
            cos3 = (L2^2+L3^2-d0^2)/(2*L2*L3);
            cos4 = (L3^2+d0^2-L2^2)/(2*L3*d0);
            Q(2) = pi-atan2(sqrt(1-cos2^2),cos2);
            Q(3) = pi/2-atan2(sqrt(1-cos3^2),cos3);
            Q(4) = -atan2(sqrt(1-cos4^2),cos4);
        end
        Q(1) = NaN(1);
        Q(5) = NaN(1);
        isPos = 1;
        q = Q;
    else if abs(r3(1)*y-r3(2)*x) < 0.001  %determine whether the orientation is feasible
            isPos = 1;
            Q = InverseK(T0e);           %do inverse kinematics
            q = Jointlimit(Q);          %check joint limits
        else
            isPos = 0;
            trans = [];
            cosinf = (x*r3(1)+y*r3(2))/(sqrt(x^2+y^2)*sqrt((r3(1)^2)+(r3(2)^2)));
            if r3(2) < y*r3(1)/x
                inf = atan2(sqrt(1-cosinf^2),cosinf);
            else inf = -atan2(sqrt(1-cosinf^2),cosinf);
            end
            trans = [cos(inf) -sin(inf) 0; sin(inf) cos(inf) 0; 0 0 1];   %rotate the axis to make the orientation feasible
            T0e(1:3,1) = trans*r1;
            T0e(1:3,2) = trans*r2;
            T0e(1:3,3) = trans*r3;
            Q = InverseK(T0e);           %do inverse kinematics
            q = Jointlimit(Q);          %check joint limits
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
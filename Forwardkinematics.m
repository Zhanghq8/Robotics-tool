%%%  Author: Hanqing Zhang 
%%%  From: WPI, Worcester, MA
%%%  Date: Oct. 6th, 2018
%%%  This code is to calculate the Forward Kinematics for 6 DOF robot arm, all conventions including the building of all   %%%
%%%  frames, DH parameters, the equation of calculating the forward kinematics matrix are following the method in "Robot Modeling %%% 
%%%  and control".

%%%  A Matrix
A1=simplify(DHmatrix(1));
A2=simplify(DHmatrix(2));
A3=simplify(DHmatrix(3));
A4=simplify(DHmatrix(4));
A5=simplify(DHmatrix(5));
A6=simplify(DHmatrix(6));

%%%  T Matrix
T1=simplify(A1);
T2=simplify(A1*A2);
T3=simplify(A1*A2*A3);
T4=simplify(A1*A2*A3*A4);
T5=simplify(A1*A2*A3*A4*A5);
T6=simplify(A1*A2*A3*A4*A5*A6);

% r11=T(1,1);
% r21=T(2,1);
% r31=T(3,1);
% r12=T(1,2);
% r22=T(2,2);
% r32=T(3,2);
% r13=T(1,3);
% r23=T(2,3);
% r33=T(3,3);
% dx=T(1,4);
% dy=T(2,4);
% dz=T(3,4);

%%%  A Matrix Function
function A = DHmatrix(i)

    syms   d1 d2 d3 d4 d5 d6 ...
        alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 ...
        a1 a2 a3 a4 a5 a6 ...
        theta1 theta2 theta3 theta4 theta5 theta6;
    
    %%% Use sym(pi/2) instead of pi/2 to simplify the result

    alpha = [alpha1,alpha2,alpha3,alpha4,alpha5,alpha6];
    d     = [d1,d2,d3,d4,d5,d6];
    a     = [a1,a2,a3,a4,a5,a6];
    theta = [theta1,theta2,theta3,theta4,theta5,theta6];

    A=[cos(theta(i)),  -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)),...
              a(i)*cos(theta(i));
        sin(theta(i)),  cos(theta(i))*cos(alpha(i)),  -cos(theta(i))*sin(alpha(i)),...
              a(i)*sin(theta(i));
        0,  sin(alpha(i)), cos(alpha(i)), d(i);
        0,  0,   0,   1];
end
%%%  Author: Hanqing Zhang 
%%%  From: WPI, Worcester, MA
%%%  Date: Oct. 6th, 2018
%%%  This code is to calculate the Jacobian Matrix for 3 DOF robot arm, all conventions including the building of all   %%%
%%%  frames, DH parameters, the equation of calculating the jacobian matrix are following the method in "Robot Modeling %%% 
%%%  and control".

%%%  A Matrix
A1=simplify(DHmatrix(1));
A2=simplify(DHmatrix(2));
A3=simplify(DHmatrix(3));

%%%  T Matrix
T1=simplify(A1);
T2=simplify(A1*A2);
T3=simplify(A1*A2*A3);

%%%  O Vector
O0=[0;0;0];
O1=[T1(1,4);T1(2,4);T1(3,4)];
O2=[T2(1,4);T2(2,4);T2(3,4)];
O3=[T3(1,4);T3(2,4);T3(3,4)];

%%%  Rotation Matrix
R01=[T1(1,1),T1(1,2),T1(1,3);
    T1(2,1),T1(2,2),T1(2,3);
    T1(3,1),T1(3,2),T1(3,3)];
R02=[T2(1,1),T2(1,2),T2(1,3);
    T2(2,1),T2(2,2),T2(2,3);
    T2(3,1),T2(3,2),T2(3,3)];
R03=[T3(1,1),T3(1,2),T3(1,3);
    T3(2,1),T3(2,2),T3(2,3);
    T3(3,1),T3(3,2),T3(3,3)];

%%%  K Vector
K=[0;0;1];

%%%  Z Vector
Z0=[0;0;1];
Z1=R01*K;
Z2=R02*K;

%%%  Linear Velocity

%%%  If joint i is revolute
%%%  Jv(i)=cross(Z(i-1),(O3-O(i-1)));
%%%  If joint i is prismatic
%%%  Jv(i)=Z(i-1);

%%%  If joint 1,2,3 are all revolute, we have
Jv1=simplify(cross(Z0,(O3-O0)));
Jv2=simplify(cross(Z1,(O3-O1)));
Jv3=simplify(cross(Z2,(O3-O2)));

%%%  Angular Velocity
%%%  If joint i is revolute
%%%  Jw(i)=Z(i-1);
%%%  If joint i is prismatic
%%%  Jw(i)=0;

%%%  If joint 1,2,3 are all revolute, we have
Jw1=Z0;
Jw2=Z1;
Jw3=Z2;

%%%  Jacobian Matrix
%%%  Jq=[Jv1, Jv2, Jv3;
%%%  Jw1, Jw2, Jw3];
Jq=[Jv1(1,1),Jv2(1,1),Jv3(1,1);
    Jv1(2,1),Jv2(2,1),Jv3(2,1);
    Jv1(3,1),Jv2(3,1),Jv3(3,1);
    Jw1(1,1),Jw2(1,1),Jw3(1,1);
    Jw1(2,1),Jw2(2,1),Jw3(2,1);
    Jw1(3,1),Jw2(3,1),Jw3(3,1)];

%%%  A Matrix Function
function A = DHmatrix(i)

    syms   d1 d2 d3 alpha1 alpha2 alpha3 a1 a2 a3 theta1 theta2 theta3;
    
%%% Use sym(pi/2) instead of pi/2 to simplify the result

    alpha = [alpha1,alpha2,alpha3];
    d     = [d1,d2,d3];
    a     = [a1,a2,a3];
    theta = [theta1,theta2,theta3];

    A=[cos(theta(i)),  -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)),...
              a(i)*cos(theta(i));
        sin(theta(i)),  cos(theta(i))*cos(alpha(i)),  -cos(theta(i))*sin(alpha(i)),...
              a(i)*sin(theta(i));
        0,  sin(alpha(i)), cos(alpha(i)), d(i);
        0,  0,   0,   1];
end
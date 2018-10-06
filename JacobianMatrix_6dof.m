%%%  Author: Hanqing Zhang 
%%%  From: WPI, Worcester, MA
%%%  Date: Oct. 6th, 2018
%%%  This code is to calculate the Jacobian Matrix for 6 DOF robot arm, all conventions including the building of all   %%%
%%%  frames, DH parameters, the equation of calculating the jacobian matrix are following the method in "Robot Modeling %%% 
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

%%%  O Vector
O0=[0;0;0];
O1=[T1(1,4);T1(2,4);T1(3,4)];
O2=[T2(1,4);T2(2,4);T2(3,4)];
O3=[T3(1,4);T3(2,4);T3(3,4)];
O4=[T4(1,4);T4(2,4);T4(3,4)];
O5=[T5(1,4);T5(2,4);T5(3,4)];
O6=[T6(1,4);T6(2,4);T6(3,4)];

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
R04=[T4(1,1),T4(1,2),T4(1,3);
    T4(2,1),T4(2,2),T4(2,3);
    T4(3,1),T4(3,2),T4(3,3)];
R05=[T5(1,1),T5(1,2),T5(1,3);
    T5(2,1),T5(2,2),T5(2,3);
    T5(3,1),T5(3,2),T5(3,3)];
R06=[T6(1,1),T6(1,2),T6(1,3);
    T6(2,1),T6(2,2),T6(2,3);
    T6(3,1),T6(3,2),T6(3,3)];

%%%  K Vector
K=[0;0;1];

%%%  Z Vector
Z0=[0;0;1];
Z1=R01*K;
Z2=R02*K;
Z3=R03*K;
Z4=R04*K;
Z5=R05*K;

%%%  Linear Velocity

%%%  If joint i is revolute
%%%  Jv(i)=cross(Z(i-1),(O3-O(i-1)));
%%%  If joint i is prismatic
%%%  Jv(i)=Z(i-1);

%%%  If joint 1-6 are all revolute, we have
Jv1=simplify(cross(Z0,(O6-O0)));
Jv2=simplify(cross(Z1,(O6-O1)));
Jv3=simplify(cross(Z2,(O6-O2)));
Jv4=simplify(cross(Z3,(O6-O3)));
Jv5=simplify(cross(Z4,(O6-O4)));
Jv6=simplify(cross(Z5,(O6-O5)));

%%%  Angular Velocity
%%%  If joint i is revolute
%%%  Jw(i)=Z(i-1);
%%%  If joint i is prismatic
%%%  Jw(i)=0;

%%%  If joint 1-6 are all revolute, we have
Jw1=Z0;
Jw2=Z1;
Jw3=Z2;
Jw4=Z3;
Jw5=Z4;
Jw6=Z5;

%%%  Jacobian Matrix
%%%  Jq=[Jv1, Jv2, Jv3, Jv4, Jv5, Jv6;
%%%  Jw1, Jw2, Jw3, Jw4, Jw5, Jw6];
Jq=[Jv1(1,1),Jv2(1,1),Jv3(1,1),Jv4(1,1),Jv5(1,1),Jv6(1,1);
    Jv1(2,1),Jv2(2,1),Jv3(2,1),Jv4(2,1),Jv5(2,1),Jv6(2,1);
    Jv1(3,1),Jv2(3,1),Jv3(3,1),Jv4(3,1),Jv5(3,1),Jv6(3,1);
    Jw1(1,1),Jw2(1,1),Jw3(1,1),Jw4(1,1),Jw5(1,1),Jw6(1,1);
    Jw1(2,1),Jw2(2,1),Jw3(2,1),Jw4(2,1),Jw5(2,1),Jw6(2,1);
    Jw1(3,1),Jw2(3,1),Jw3(3,1),Jw4(3,1),Jw5(3,1),Jw6(3,1)];

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
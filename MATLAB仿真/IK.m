%% OUR_Manipulator
%机械臂建模
clear;
clc;
% DH表
theta1 = 0; d1 = 0;   a1 = 0;    alpha1 = -pi/2;  offset1 = 0;
theta2 = 0; d2 = 0;   a2 = 250;  alpha2 = 0;      offset2 = -pi/2;
theta3 = 0; d3 = 0;   a3 = 250;  alpha3 = 0;      offset3 = pi/2;
theta4 = 0; d4 = 0;   a4 = 0;    alpha4 = -pi/2;  offset4 = 0;
theta5 = 0; d5 = 0;   a5 = 0;    alpha5 = 0;      offset5 = 0;
% 创建关节


L(1) = Link('revolute','d',d1,'a',a1,'alpha', alpha1,'offset', offset1);
L(2) = Link('revolute','d',d2,'a',a2,'alpha', alpha2,'offset', offset2);
L(3) = Link('revolute','d',d3,'a',a3,'alpha', alpha3,'offset', offset3);
L(4) = Link('revolute','d',d4,'a',a4,'alpha', alpha4,'offset', offset4);
L(5) = Link('revolute','d',d5,'a',a5,'alpha', alpha5,'offset', offset5);
Five_dof=SerialLink(L,'name','5-dof');


%查看DH表
Five_dof.display();

%查看GUI
Five_dof.teach 

%% 计算反矩阵，能够获得方程式左边的表达式
clear
syms a2 a3 a4
syms d1 d2 d3 d4 d5 
syms theta1 theta2 theta3 theta4 theta5
%矩阵T01
T01 = [cos(theta1),  0, -sin(theta1),  0;
       sin(theta1),  0,  cos(theta1),  0;
                 0, -1,            0,  0;
                 0,  0,            0,  1]
%矩阵R01
R01 = [cos(theta1),  0, -sin(theta1);
       sin(theta1),  0,  cos(theta1);
                 0, -1,            0]
%向量P01org
P01 = [0;0;0]

%矩阵R01转置
R01t = R01.'

%逆矩阵
T01inv = [R01t,-R01t*P01;
          0, 0, 0,1]

%等式左边inv（T01）*T05
syms r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz
T05 = [r11 r12 r13 px;
       r21 r22 r23 py;
       r31 r32 r33 pz;
       0   0   0   1];

Tleft = T01inv*T05
%% 计算等式右边的T15
%clear
%clc
syms a1 a2 a3 a4 a5
syms d1 d2 d3 d4 d5 
syms theta1 theta2 theta3 theta4 theta5
syms alpha1 alpha2 alpha3 alpha4 alpha5
%% 
 % 因为程序中用的是cospi(),所以这里用1/2来表示pi/2
d1 = 0;   a1 = 0;    alpha1 = -1/2;   offset1 = 0;
d2 = 0;              alpha2 = 0;      offset2 = -pi/2;
d3 = 0;              alpha3 = 0;      offset3 = pi/2;
d4 = 0;   a4 = 0;    alpha4 = -1/2;   offset4 = 0;
d5 = 0;   a5 = 0;    alpha5 = 0;      offset5 = 0;


T01 = [cos(theta1) -cospi(alpha1)*sin(theta1)  sinpi(alpha1)*sin(theta1) a1*cos(theta1);
       sin(theta1)  cospi(alpha1)*cos(theta1) -sinpi(alpha1)*cos(theta1) a1*sin(theta1);
       0            sinpi(alpha1)              cospi(alpha1)             d1;
       0            0                        0                       1]

%theta的计算要添加在这里
T12 = [cos(theta2-pi/2) -cos(alpha2)*sin(theta2-pi/2)  sin(alpha2)*sin(theta2-pi/2) a2*cos(theta2-pi/2);
       sin(theta2-pi/2)  cos(alpha2)*cos(theta2-pi/2) -sin(alpha2)*cos(theta2-pi/2) a2*sin(theta2-pi/2);
       0                 sin(alpha2)                  cos(alpha2)                   d2;
       0                 0                            0                             1]

T23 = [cos(theta3+pi/2) -cos(alpha3)*sin(theta3+pi/2)  sin(alpha3)*sin(theta3+pi/2) a3*cos(theta3+pi/2);
       sin(theta3+pi/2)  cos(alpha3)*cos(theta3+pi/2) -sin(alpha3)*cos(theta3+pi/2) a3*sin(theta3+pi/2);
       0                 sin(alpha3)                   cos(alpha3)                  d3;
       0                 0                             0                            1]
  
T34 = [cos(theta4) -cospi(alpha4)*sin(theta4)  sinpi(alpha4)*sin(theta4) a4*cos(theta4);
       sin(theta4)  cospi(alpha4)*cos(theta4) -sinpi(alpha4)*cos(theta4) a4*sin(theta4);
       0            sinpi(alpha4)              cospi(alpha4)             d4;
       0            0                        0                       1]
T45 = [cos(theta5) -cos(alpha5)*sin(theta5)  sin(alpha5)*sin(theta5) a5*cos(theta5);
       sin(theta5)  cos(alpha5)*cos(theta5) -sin(alpha5)*cos(theta5) a5*sin(theta5);
       0            sin(alpha5)              cos(alpha5)             d5;
       0            0                        0                       1]
T15 = T12*T23*T34*T45 %最终的T15矩阵

theta = [theta1 theta2 theta3 theta4 theta5];
a =     [0 250 250 116 0];
d =     [250 0 0 0 250];
Tleft= get_left(theta,d,a)
T    = T15_calc(theta,d,a)
%% T0inv*T05 = T15
% 验证函数是否正确（当等式左边=等式右边的时候，等号成立）
theta  = [0 0 0 0 0];
a      = [0 250 250 116 0];
d      = [250 0 0 0 250];

Tinv = Get_Tinv(theta,d,a)
T05  = FK_calc(theta,d,a)
Tl   =  Tinv*T05

T15  = T15_calc(theta,d,a)
Tleft= get_left(theta,d,a)
T05  = FK_calc(theta,d,a)
%% 函数定义
clc
theta = [pi/2 pi/6 pi/7 -pi/2 0];
a =     [0 250 250 0 0];
d =     [250 0 0 0 250];
q1 = [ 0    2.0944    0.0000   -1.0472         0];
q2 = [ 0    2.0944         0   -1.0472         0];
q3 = [ 0    0.5236    0.0000    0.5236         0];
q4 = [ 0    0.5236         0    0.5236         0];

T05  = FK_calc(theta,d,a)
ik   = DOF5_ikine(T05)
Ttest = theta
T05  = FK_calc(Ttest,d,a)
Five_dof.plot(Ttest)

% 获得T05===========================================================================================
theta = [0 0 0 0 0];
a =     [0 250 250 0 0];
d =     [0 0 0 0 0];
T = FK_calc(theta,d,a);

function Get_T05 = FK_calc(theta,d,a)
c1 = cos(theta(1));c2 = cos(theta(2)-pi/2);c3 = cos(theta(3)+pi/2);c4 = cos(theta(4));c5 = cos(theta(5));
s1 = sin(theta(1));s2 = sin(theta(2)-pi/2);s3 = sin(theta(3)+pi/2);s4 = sin(theta(4));s5 = sin(theta(5));
a1 = a(1);a2 = a(2);a3 = a(3);a4 = a(4);a5 = a(5);
d1 = d(1);d2 = d(2);d3 = d(3);d4 = d(4);d5 = d(5);

r11 = c5*(c4*(c1*c2*c3-c1*s2*s3)-s4*(c1*c2*s3+c1*c3*s2))+s1*s5;%
r21 = c5*(c4*(c2*c3*s1-s1*s2*s3)-s4*(c2*s1*s3+c3*s1*s2))-c1*s5;%
r31 = -c5*(c4*(c2*s3+c3*s2)+s4*(c2*c3-s2*s3));%

r12 =  c5*s1-s5*(c4*(c1*c2*c3-c1*s2*s3)-s4*(c1*c2*s3+c1*c3*s2));
r22 = -c1*c5-s5*(c4*(s1*c2*c3-s1*s2*s3)-s4*(c2*s1*s3+c3*s1*s2));
r32 = s5*(c4*(c2*s3+s2*c3)+s4*(c2*c3-s2*s3));
 
r13 = -c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*c3-c1*s2*s3);
r23 = -c4*(c2*s1*s3+c3*s1*s2)-s4*(s1*c2*c3-s1*s2*s3);
r33 = -c4*(c2*c3-s2*s3)+s4*(c2*s3+c3*s2);

px =   a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3;
py =   a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3;
pz =  -a2*s2    - a3*c2*s3    - a3*c3*s2;

Get_T05 = [r11 r12 r13 px;r21 r22 r23 py ;r31 r32 r33 pz;0 0 0 1];
end


% 等式左邊的矩陣====================================================================================
function ans = get_left(theta,d,a)
c1 = cos(theta(1));c2 = cos(theta(2)-pi/2);c3 = cos(theta(3)+pi/2);c4 = cos(theta(4));c5 = cos(theta(5));
s1 = sin(theta(1));s2 = sin(theta(2)-pi/2);s3 = sin(theta(3)+pi/2);s4 = sin(theta(4));s5 = sin(theta(5));
a1 = a(1);a2 = a(2);a3 = a(3);a4 = a(4);a5 = a(5);
d1 = d(1);d2 = d(2);d3 = d(3);d4 = d(4);d5 = d(5);

r11 = c5*(c4*(c1*c2*c3-c1*s2*s3)-s4*(c1*c2*s3+c1*c3*s2))+s1*s5;%
r21 = c5*(c4*(c2*c3*s1-s1*s2*s3)-s4*(c2*s1*s3+c3*s1*s2))-c1*s5;%
r31 = -c5*(c4*(c2*s3+c3*s2)+s4*(c2*c3-s2*s3));%

r12 =  c5*s1-s5*(c4*(c1*c2*c3-c1*s2*s3)-s4*(c1*c2*s3+c1*c3*s2));
r22 = -c1*c5-s5*(c4*(s1*c2*c3-s1*s2*s3)-s4*(c2*s1*s3+c3*s1*s2));
r32 = s5*(c4*(c2*s3+s2*c3)+s4*(c2*c3-s2*s3));
 
r13 = -c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*c3-c1*s2*s3);
r23 = -c4*(c2*s1*s3+c3*s1*s2)-s4*(s1*c2*c3-s1*s2*s3);
r33 = -c4*(c2*c3-s2*s3)+s4*(c2*s3+c3*s2);

px =   a2*c1*c2 + a3*c1*c2*c3 - a3*c1*s2*s3;
py =   a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3;
pz =  -a2*s2    - a3*c2*s3    - a3*c3*s2;

ans = [r11*c1+r21*s1 r12*c1+r22*s1 r13*c1+r23*s1 px*c1+py*s1;
       -r31          -r32          -r33          -pz        ;
       r21*c1-r11*s1 r22*c1-r12*s1 r23*c1-r13*s1 py*c1-px*s1;
       0             0             0             1          ]

end

% 获得T01inv==========================================================================================
function Tinv = Get_Tinv(theta,d,a)
c1 = cos(theta(1));c2 = cos(theta(2)+pi/2);c3 = cos(theta(3)+pi/2);c4 = cos(theta(4));c5 = cos(theta(5));
s1 = sin(theta(1));s2 = sin(theta(2)+pi/2);s3 = sin(theta(3)+pi/2);s4 = sin(theta(4));s5 = sin(theta(5));
a1 = a(1);a2 = a(2);a3 = a(3);a4 = a(4);a5 = a(5);
d1 = d(1);d2 = d(2);d3 = d(3);d4 = d(4);d5 = d(5);

%矩阵T01
T01 = [c1,  0, -s1,  0;
       s1,  0,  c1,  0;
        0, -1,   0,  0;
        0,  0,   0,  1]
%矩阵R01
R01 = [c1,  0, -s1;
       s1,  0,  c1;
        0, -1,   0]
%向量P01org
P01 = [0;0;0]

%矩阵R01转置
R01t = R01.';

%逆矩阵
Tinv = [R01t   ,-R01t*P01;
        0, 0, 0,        1];
end

% 获得T15，也就是等式右边的表达式=========================================================================================
function Get_T15 = T15_calc(theta,d,a)
c1 = cos(theta(1));c2 = cos(theta(2)-pi/2);c3 = cos(theta(3)+pi/2);c4 = cos(theta(4));c5 = cos(theta(5));
s1 = sin(theta(1));s2 = sin(theta(2)-pi/2);s3 = sin(theta(3)+pi/2);s4 = sin(theta(4));s5 = sin(theta(5));
a1 = a(1);a2 = a(2);a3 = a(3);a4 = a(4);a5 = a(5);
d1 = d(1);d2 = d(2);d3 = d(3);d4 = d(4);d5 = d(5);

q11 = c5*(c4*(c2*c3-s2*s3)-s4*(c2*s3+c3*s2));
q21 = c5*(c4*(c2*s3+c3*s2)+s4*(c2*c3-s2*s3));
q31 = -s5;

q12 = -s5*(c4*(c2*c3-s2*s3)-s4*(c2*s3+c3*s2));
q22 = -s5*(c4*(c2*s3+c3*s2)+s4*(c2*c3-s2*s3));
q32 = -c5;

q13 = -c4*(c2*s3+c3*s2)-s4*(c2*c3-s2*s3);
q23 =  c4*(c2*c3-s2*s3)-s4*(c2*s3+c3*s2);
q33 = 0;

Qx = a2*c2 + a3*c2*c3 - a3*s2*s3;
Qy = a2*s2 + a3*c2*s3 + a3*c3*s2;
Qz = 0;

Get_T15 = [q11 q12 q13 Qx;q21 q22 q23 Qy;q31 q32 q33 Qz;0 0 0 1];
end

%计算逆解
%======================================================================================================================
function theta_ikine = DOF5_ikine(T)

%机械臂参数
a2 = 250; a3 = 250;
%T06矩阵
r11 = T(1, 1);r12 = T(1, 2);r13 = T(1, 3);px = T(1, 4);
r21 = T(2, 1);r22 = T(2, 2);r23 = T(2, 3);py = T(2, 4);
r31 = T(3, 1);r32 = T(3, 2);r33 = T(3, 3);pz = T(3, 4);


%theta1 一个解
theta1 = atan2(py,px);
c1 = cos(theta1);
s1 = sin(theta1);
%theta5 一个解
theta5 = atan2(- r21*c1 + r11*s1,- r22*c1 + r12*s1);

%theta3 两个解
c3 = (pz^2 +px^2*c1^2+py^2*s1^2+2*px*py*c1*s1-a2^2-a3^2)/(2*a2*a3);
s3 = sqrt(1-c3^2);
theta3_1 =  atan2(s3,+c3);
theta3_2 =  atan2(s3,-c3);
%theta2 两个解
pm  = px*c1 + py*s1;
R   = pm^2+a2^2+pz^2-a3^2;
Ro  = sqrt((2*pm*a2)^2+(2*pz*a2)^2);
Phi = atan2(2*pm*a2,2*pz*a2);
theta2_1 =  atan2(R/Ro,sqrt(1-(R/Ro)^2))-Phi;
theta2_2 =  atan2(R/Ro,-sqrt(1-(R/Ro)^2))-Phi;

a =  2*pm*a2;
b =  2*pz*a2;
c =  pm^2+a2^2+pz^2-a3^2;

theta2_1 = atan2(b,a)+atan2(sqrt(a^2+b^2-c^2),c);
theta2_2 = atan2(b,a)-atan2(sqrt(a^2+b^2-c^2),c);

%theta4 四个解
c234 = -r33;
s234 = sqrt(1-r33^2);
theta4_1 = atan2(s234,c234) -theta2_1 - theta3_1;
theta4_2 = atan2(s234,c234) -theta2_1 - theta3_2;
theta4_3 = atan2(s234,c234) -theta2_2 - theta3_1;
theta4_4 = atan2(s234,c234) -theta2_2 - theta3_2;

theta_ikine = [theta1 theta2_1+pi/2 theta3_1-pi/2 theta4_1 theta5;

               theta1 theta2_1+pi/2 theta3_2-pi/2 theta4_2 theta5;
               theta1 theta2_2+pi/2 theta3_1-pi/2 theta4_3 theta5;
               theta1 theta2_2+pi/2 theta3_2-pi/2 theta4_4 theta5];

end
























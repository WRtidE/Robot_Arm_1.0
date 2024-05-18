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

% 关节角度限制
L(1).qlim = [-150,150]/180 * pi;
L(2).qlim = [-70,90] /180 * pi;
L(3).qlim = [-70,10]  /180 * pi;
L(4).qlim = [-90,90]/180 * pi;
L(5).qlim = [-180,180]/180 * pi;

%查看DH表
Five_dof.display();

%查看GUI
Five_dof.teach 


%% 获得数值解



alpha1 = -1/2;  % 因为程序中用的是cospi(),所以这里用1/2来表示pi/2
alpha2 = 0;
alpha3 = 0;
alpha4 = -1/2;
alpha5 = 0;

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
T05 = T01*T12*T23*T34*T45 %最终的T矩阵

q0  = [theta1 theta2 theta3 theta4 theta5];
T1 = Five_dof.fkine(q0)

%函数验证
theta = [theta1 theta2 theta3 theta4 theta5];
a =     [0 250 250 0 0];
d =     [0 0 0 0 0];
T2 = FK_calc(theta,d,a)
T3 = Five_dof.fkine(theta)
%% 获得表达式，方便写成函数

%clear
%clc
syms a1 a2 a3 a4 a5
syms d1 d2 d3 d4 d5 
syms theta1 theta2 theta3 theta4 theta5
syms alpha1 alpha2 alpha3 alpha4 alpha5

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
T05 = T01*T12*T23*T34*T45 %最终的T矩阵

%% 函数定义
theta = [0 pi/2 0 0 0];
a =     [0 250 250 0 0];
d =     [0 0 0 0 0];
T = FK_calc(theta,d,a)

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


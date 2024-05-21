clc;clear;close all
% 设置角度单位转换
degtorad = pi/180;

l = 0;

% 改进型DH表
theta1 = 0; d1 = 63;  a1 = 0;    alpha1 =   0 * degtorad;   
theta2 = 0; d2 = 0;   a2 = 0;    alpha2 =  90 * degtorad;  
theta3 = 0; d3 = 0;   a3 = 250;  alpha3 =   0 * degtorad;  
theta4 = 0; d4 = 0;   a4 = 250;  alpha4 =   0 * degtorad; 
theta5 = 0; d5 = 116; a5 = 0;    alpha5 = -90 * degtorad;

% 设置连杆偏距
d = [d1,d2,d3,d4,d5];
% 设置连杆长度
a = [a1,a2,a3,a4,a5];
%设置连杆扭矩角
alpha = [alpha1, alpha2,alpha3,alpha4,alpha5];

L(1)=Link([ 0   d1   a1   alpha1], 'modified');
L(2)=Link([ 0   d2   a2   alpha2], 'modified');L(2).offset = pi/2; 
L(3)=Link([ 0   d3   a3   alpha3], 'modified');L(3).offset = pi/2; 
L(4)=Link([ 0   d4   a4   alpha4], 'modified');L(4).offset = -pi/2; 
L(5)=Link([ 0   d5   a5   alpha5], 'modified');

robot=SerialLink(L,'name','robot');
robot.teach()

%q1 = [0,0,0,pi/2,0];
q1 =  [90,-7,0,-50,-7]*pi/180;

T1 = robot.fkine(q1)
T2 = FK(q1,a,d,alpha,l)


theta = IK(T2,a,d,l)

for i = 1:4
    %robot.plot(theta(i,:));
   % pause;
%     T = robot.fkine(theta(i,:));
%      disp(T.t)
end


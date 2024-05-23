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

%% FK.m 正解函数测试
q1 = [0,0,0,pi/2,0];
%q1 =  [90,-7,0,-50,-7]*pi/180;
T1 = robot.fkine(q1)
T2 = FK(q1,a,d,alpha,l)

%% IK., 逆解函数测试
theta = IK(T2,a,d,l)
robot.plot(theta(1,:));
pause;

%% target_calc.m 函数测试，输入xyz的值，让机械臂执行器达到指定位置
Txyz = target_calc(155,269,0);
theta = IK(Txyz,a,d,l)
robot.plot(theta(1,:))
pause;
Txyz = target_calc(-155,-269,20);
theta = IK(Txyz,a,d,l)
robot.plot(theta(1,:))
pause;


%% path_calc 函数测试,输入目标角度和当前角度等信息，获得轨迹规划
theta_start = [0 0 0 0 0];
theta_final = [pi/2 pi/6 pi/7 -pi/2 0];
v_s = [0 0 0 0 0];
v_f = [0 0 0 0 0];
a_s = [0 0 0 0 0];
a_f = [0 0 0 0 0];
t   = [0 10];

T = path_calc(theta_start,theta_final,v_s,v_f,a_s,a_f,t);
q1 = T.motor1.theta;
q2 = T.motor2.theta;
q3 = T.motor3.theta;
q4 = T.motor4.theta;
q5 = T.motor5.theta;

%轨迹规划仿真
num = 1;
for i = 1:numel(q1)-1
    B = [q1(num) q2(num) q3(num) q4(num) q5(num)]
    num = num+1
    subplot(3,2,6)
    robot.plot(B)
end
%% 综合测试

%输入目标x y z 坐标值
box.x = -155;
box.y = -269;
h     = 10;

%计算出物块相对于基坐标的矩阵
Tbox_robo = target_calc(box.x,box.y,h);

%逆解出角度
theta = IK(Tbox_robo,a,d,l)


% 判断是否越界,从四组解中找到一组最适合的解，主要分析关节1/2/3即可
for i = 1:4
 i
 theta(i,1)
 if((-2<theta(i,1))&&(theta(i,1)<2) ...
  && (-1< theta(i,2))&& ( theta(i,2)< 1) ...
  && (-1.67<theta(i,3))&&(theta(i,3)<0.9))

  robot.plot(theta(i,:))
  theta_final = theta(i,:)% 目标位置的四个转角

 end
 pause;
end

%轨迹规划
theta_start= [0 0 0 0 0];
v_s = [0 0 0 0 0];
v_f = [0 0 0 0 0];
a_s = [0 0 0 0 0];
a_f = [0 0 0 0 0];
t   = [0 10];

T = path_calc(theta_start,theta_final,v_s,v_f,a_s,a_f,t);
q1 = T.motor1.theta;
q2 = T.motor2.theta;
q3 = T.motor3.theta;
q4 = T.motor4.theta;
q5 = T.motor5.theta;

% 绘制运动图
num = 1;
for i = 1:numel(q1)-1
    B = [q1(num) q2(num) q3(num) q4(num) q5(num)]
    num = num+1
    robot.plot(B)
end






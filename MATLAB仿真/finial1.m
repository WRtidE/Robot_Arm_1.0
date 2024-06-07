%%物块搬运仿真
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
robot.display();



%% 综合测试
cla
%机械臂初始化
theta_start = [0 0 0 pi/2 0];
v_s = [0 0 0 0 0];
v_f = [0 0 0 0 0];
a_s = [0 0 0 0 0];
a_f = [0 0 0 0 0];
t   = [0 5];
Ttool_box = [0 1 0 0; 1 0 0 0; 0 0 -1 63+15; 0 0 0 1];

%放一个小球
plot_sphere([-155,-269,10],30)

robot.plot(theta_start)
pause;
%=======抓取=======
%发现目标
%输入目标x y z 坐标值
box.x = -155;
box.y = -269;
h     = 10;

%计算出物块相对于基坐标的矩阵
Tbox_robo = target_calc(box.x,box.y,h);

%逆解出角度
theta = IK(Tbox_robo,a,d,l);


% 判断是否越界,从四组解中找到一组最适合的解，主要分析关节1/2/3即可
for i = 1:4
 theta(i,1)
 if((-2<theta(i,1))&&(theta(i,1)<2) ...
  && (-1< theta(i,2))&& ( theta(i,2)< 1) ...
  && (-1.67<theta(i,3))&&(theta(i,3)<0.9))
  theta_final = theta(i,:)% 目标位置的四个转角
 end
end

%规划路径
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
    T0_tool = FK(B,a,d,alpha,l);
    T0_box  = T0_tool*Ttool_box;
    P = transl(T0_box)
    %%plot_sphere(P,20)
    robot.plot(B,"fps",60)
end


%%======via_points========
theta_start = theta_final;
theta_final = [0 0 0 pi/2 0]
t   = [5 10];

%规划路径
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
    T0_tool = FK(B,a,d,alpha,l);
    T0_box  = T0_tool*Ttool_box;
    P = transl(T0_box)
    plot_sphere(P,2)
    robot.plot(B,"fps",60)
end

plot_sphere(P,30)

%%===========放置=============
theta_start = theta_final;
t   = [10 15];
%输入目标x y z 坐标值
box.x = -155;
box.y = 269;
h     = 10;

%计算出物块相对于基坐标的矩阵
Tbox_robo = target_calc(box.x,box.y,h);

%逆解出角度
theta = IK(Tbox_robo,a,d,l);


% 判断是否越界,从四组解中找到一组最适合的解，主要分析关节1/2/3即可
for i = 1:4
 theta(i,1)
 if((-2<theta(i,1))&&(theta(i,1)<2) ...
  && (-1< theta(i,2))&& ( theta(i,2)< 1) ...
  && (-1.67<theta(i,3))&&(theta(i,3)<0.9))
  %robot.plot(theta(i,:))
  theta_final = theta(i,:)% 目标位置的四个转角
 end
end

%规划路径
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
    T0_tool = FK(B,a,d,alpha,l);
    T0_box  = T0_tool*Ttool_box;
    P = transl(T0_box)
    plot_sphere(P,2)
    robot.plot(B,"fps",60)
end

plot_sphere(P,30)
%%======回来========
theta_start = theta_final;
theta_final = [0 0 0 pi/2 0]
t   = [5 10];

%规划路径
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
    T0_tool = FK(B,a,d,alpha,l);
    T0_box  = T0_tool*Ttool_box;
    P = transl(T0_box)
    plot_sphere(P,2)
    robot.plot(B,"fps",60)
end

plot_sphere(P,30)

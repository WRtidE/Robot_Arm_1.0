%% c语言验证
 x1=[0,1,2,3,4,5,6,7,8,9,10];
 y1=[10,10.68,14.63,23.046,35.39,49.9,64.6,76.9,85.36,89.31,89.9];%这个部分替换成—你的数据，注意一一对应
 plot(x1,y1);
 semilogy(x1,y1)%原来的折线figure 1


x2=linspace(min(x1),max(x1));
y2=interp1(x1,y1,x2,'cubic');
figure
semilogy(x2,y2)%处理后的曲线figure 2

 x1=linspace(min(x),max(x));
y1=interp1(x,y,x1,'cubic');

plot(x1,y1);

%% OUR_Manipulator
%机械臂建模
clear;
clc;
% DH表
theta1 = 0; D1 = 0;   A1 = 0;    alpha1 = -pi/2;  offset1 = 0;
theta2 = 0; D2 = 0;   A2 = 250;  alpha2 = 0;      offset2 = -pi/2;
theta3 = 0; D3 = 0;   A3 = 250;  alpha3 = 0;      offset3 = pi/2;
theta4 = 0; D4 = 0;   A4 = 0;    alpha4 = -pi/2;  offset4 = 0;
theta5 = 0; D5 = 0;   A5 = 0;    alpha5 = 0;      offset5 = 0;
% 创建关节


L(1) = Link('revolute','d',D1,'a',A1,'alpha', alpha1,'offset', offset1);
L(2) = Link('revolute','d',D2,'a',A2,'alpha', alpha2,'offset', offset2);
L(3) = Link('revolute','d',D3,'a',A3,'alpha', alpha3,'offset', offset3);
L(4) = Link('revolute','d',D4,'a',A4,'alpha', alpha4,'offset', offset4);
L(5) = Link('revolute','d',D5,'a',A5,'alpha', alpha5,'offset', offset5);
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

%% draft
theta_start = [0 0 0 0 0];
theta_final = [pi/2 pi/6 pi/7 -pi/2 0];
v_s = [0 0 0 0 0];
v_f = [0 0 0 0 0];
a_s = [0 0 0 0 0];
a_f = [0 0 0 0 0];
t   = [0 10];

T = path_calc(theta_start,theta_final,v_s,v_f,a_s,a_f,t);
q1 = T.motor1.theta
q2 = T.motor2.theta
q3 = T.motor3.theta
q4 = T.motor4.theta
q5 = T.motor5.theta

%轨迹规划仿真
num = 1;
for i = 1:numel(q1)-1
    B = [q1(num) q2(num) q3(num) q4(num) q5(num)]
    num = num+1
    subplot(3,2,6)
    Five_dof.plot(B)
end

%% how to use for
clc
num = 1
N = size(A)
N(2)
A = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15];
for i = 1:1:N(2)
   i
end

for i = 0:5:N(2)-1
    B = [A(num) A(num+1) A(num+2) A(num+3) A(num+4)]
    num = num + 5;
end


%% 
% theta_start = [10 0 0 0 0];
% theta_final = [90 0 0 0 0];
% v_s = [0 0 0 0 0];
% v_f = [0 0 0 0 0];
% a_s = [0 0 0 0 0];
% a_f = [0 0 0 0 0];
% t   = [0 10];
% 
% path_calc(theta_start,theta_final,v_s,v_f,a_s,a_f,t);

% theta_start = [45 180 90 0 23];
% theta_final = [90 180 120 20 73];
% v_s = [0 0 0 0 0];
% v_f = [0 0 0 0 0];
% a_s = [0 0 0 0 0];
% a_f = [0 0 0 0 0];
% t   = [10 20];
% path_calc(theta_start,theta_final,v_s,v_f,a_s,a_f,t);
theta = [10,90];
v = [0,0];
a = [0,0];
t = [5,10];
a_path_calc(theta,v,a,t)

% 轨迹规划函数 两点之间确定一条直线===========================================
function res = a_path_calc(theta,v,a,t)

% 起始角度和终止角度
q0     = theta(1);
qf     = theta(2);
dq0    = v(1);
dqf    = v(2);
ddq0   = a(1);
ddqf   = a(2);
ts     = t(1);
tf     = t(2);
% 路径规划公式
%q0   = theta0   = c0
%qf   = thetaf   = c0 + c1*tf +c2*tf^2 +c3*tf^3 + c4*tf^4 + c5*tf^5
%dq0  = dtheta0  = c1
%dqf  = dthetaf  = c1 + 2*c2*tf + 3*c3*tf^2 + 4*c4*tf^3 + 5*c5*tf^4
%ddq0 = ddtheta0 = 2*c2 
%ddqf = ddthetaf = 2*c2 + 6*c3*tf +12*c4*tf^2 +20*c5*tf^3

% 由路径规划公式推出下面计算多项式的各项系数
% 计算五次多项式
c0 = q0;
c1 = dq0;
c2 = ddq0/2;
c3 = (20*(qf-q0)-( 8*dqf + 12*dq0)*(tf-ts) - (3*ddq0 -   ddqf)*(tf-ts)^2)/(2*(tf-ts)^3);
c4 = (30*(q0-qf)+(14*dqf + 16*dq0)*(tf-ts) + (3*ddq0 - 2*ddqf)*(tf-ts)^2)/(2*(tf-ts)^4);
c5 = (12*(qf-q0)-( 6*dqf +  6*dq0)*(tf-ts) - (  ddq0 -   ddqf)*(tf-ts)^2)/(2*(tf-ts)^5);

%获得公式
t = ts:0.1:tf;
%1 角度  五次多项式
theta =c0+c1*power((t-ts),1)+c2*power((t-ts),2)+c3*power((t-ts),3)+c4*power((t-ts),4)+c5*power((t-ts),5)
%2 速度  五次多项式
vel   =c1+2*c2*power((t-ts),1)+3*c3*power((t-ts),2)+4*c4*power((t-ts),3)+5*c5*power((t-ts),4);
%3 加速度五次多项式
acc   =2*c2+6*c3*power((t-ts),1)+12*c4*power((t-ts),2)+20*c5*power((t-ts),3)



% % 绘制theta路径曲线
% subplot(3,1,1)
% plot(t,theta,'r','linewidth',2)
% ylabel('position')
% grid on
% hold on;
% % 绘制vel速度曲线
% subplot(3,1,2)
% plot(t,vel,'g','linewidth',2)
% ylabel('velocity')
% grid on
% hold on;
% %绘制加速度曲线
% subplot(3,1,3)
% plot(t,acc,'b','linewidth',2)
% ylabel('acceleration')
% grid on
% hold on;
motor.theta = theta;
motor.vel   = vel;
motor.acc   = acc;

res = motor;
end



% 整体路径规划 根据起始角度和终止角度得出5个轴的运动轨迹
function path = path_calc(theta_start,theta_final,v_s,v_f,a_s,a_f,t)

% 起始角度和终止角度
q1_s = theta_start(1);q1_f = theta_final(1);
q2_s = theta_start(2);q2_f = theta_final(2);
q3_s = theta_start(3);q3_f = theta_final(3);
q4_s = theta_start(4);q4_f = theta_final(4);
q5_s = theta_start(5);q5_f = theta_final(5);

%起始速度和最终速度
v1_s = v_s(1) ; v1_f = v_f(1);
v2_s = v_s(2) ; v2_f = v_f(2);
v3_s = v_s(3) ; v3_f = v_f(3);
v4_s = v_s(4) ; v4_f = v_f(4);
v5_s = v_s(5) ; v5_f = v_f(5);

%起始加速度和最终加速度
a1_s = a_s(1) ; a1_f = a_f(1);
a2_s = a_s(2) ; a2_f = a_f(2);
a3_s = a_s(3) ; a3_f = a_f(3);
a4_s = a_s(4) ; a4_f = a_f(4);
a5_s = a_s(5) ; a5_f = a_f(5);

% 每个轴的轨迹
motor1 = a_path_calc([q1_s,q1_f],[v1_s,v1_f],[a1_s,a1_f],t);
motor2 = a_path_calc([q2_s,q2_f],[v2_s,v2_f],[a2_s,a2_f],t);
motor3 = a_path_calc([q3_s,q3_f],[v3_s,v3_f],[a3_s,a3_f],t);
motor4 = a_path_calc([q4_s,q4_f],[v4_s,v4_f],[a4_s,a4_f],t);
motor5 = a_path_calc([q5_s,q5_f],[v5_s,v5_f],[a5_s,a5_f],t);

%画图
t = t(1):0.1:t(2);
% 绘制motor1路径曲线
subplot(3,2,1)
ylabel('motor1')
plot(t,motor1.theta,'r','linewidth',2)
hold on;
plot(t,motor1.vel,'g','linewidth',2)
hold on;
plot(t,motor1.acc,'b','linewidth',2)
grid on;
hold on;

% 绘制motor2路径曲线
subplot(3,2,2)
ylabel('motor2')
plot(t,motor2.theta,'r','linewidth',2)
hold on;
plot(t,motor2.vel,'g','linewidth',2)
hold on;
plot(t,motor2.acc,'b','linewidth',2)
grid on;
hold on;


% 绘制motor3路径曲线
subplot(3,2,3)
ylabel('motor3')
plot(t,motor3.theta,'r','linewidth',2)
hold on;
plot(t,motor3.vel,'g','linewidth',2)
hold on;
plot(t,motor3.acc,'b','linewidth',2)
grid on;
hold on;

% 绘制motor4路径曲线
subplot(3,2,4)
ylabel('motor4')
plot(t,motor4.theta,'r','linewidth',2)
hold on;
plot(t,motor4.vel,'g','linewidth',2)
hold on;
plot(t,motor4.acc,'b','linewidth',2)
grid on;
hold on;

% 绘制motor5路径曲线
subplot(3,2,5)
ylabel('motor5')
plot(t,motor5.theta,'r','linewidth',2)
hold on;
plot(t,motor5.vel,'g','linewidth',2)
hold on;
plot(t,motor5.acc,'b','linewidth',2)
grid on;
hold on;

%传回结构体
res.motor1 = motor1;
res.motor2 = motor2;
res.motor3 = motor3;
res.motor4 = motor4;
res.motor5 = motor5;

path = res;

end



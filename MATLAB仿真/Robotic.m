%% Our——manipulator
clear;
clc;
theta1 = 0; D1 = 250; A1 = 0;    alpha1 = pi/2;  offset1 = pi/2;
theta2 = 0; D2 = 0;   A2 = 250;  alpha2 = 0;     offset2 = pi/2;
theta3 = 0; D3 = 0;   A3 = 250;  alpha3 = 0;     offset3 = pi/2;
theta4 = 0; D4 = -45; A4 = 116;  alpha4 = -pi/2; offset4 = 0;
theta5 = 0; D5 = 250; A5 = 0;    alpha5 = 0;     offset5 = 0;


L(1) = Link('revolute','d',D1,'a',A1,'alpha', alpha1,'offset', offset1);
L(2) = Link('revolute','d',D2,'a',A2,'alpha', alpha2,'offset', offset2);
L(3) = Link('revolute','d',D3,'a',A3,'alpha', alpha3,'offset', offset3);
L(4) = Link('revolute','d',D4,'a',A4,'alpha', alpha4,'offset', offset4);
L(5) = Link('revolute','d',D5,'a',A5,'alpha', alpha5,'offset', offset5);

Five_dof=SerialLink(L,'name','5-dof');
Five_dof.base = transl(0,0,0);

L(1).qlim = [-150,150]/180 * pi;
L(2).qlim = [-120,70] /180 * pi;
L(3).qlim = [-120,10]  /180 * pi;
L(4).qlim = [-90,90]/180 * pi;
L(5).qlim = [-180,180]/180 * pi;

Five_dof.display();  %查看DH表
Five_dof.teach       %查看GUI
%% 旋转-->旋转矩阵
clear;
clc;
%Eular Angle
T1 = eul2tr(90,60,30) 
T2 = trotz(90)*troty(60)*trotz(30)
%Fixed Angle
T3 = rpy2tr(90,60,30)
T4 = trotz(30)*troty(60)*trotx(90)
%% 位移-->变换矩阵
T  =  transl(1.5,1,0.5)*trotx(30)*trotz(60) %get a transformation matrix
P  =  transl(T)  % get P from T
R  =  t2r(T)     % get R from T
T1 =  r2t(R)     % get T from R 
%% 机器人工具箱常用函数
Five_dof.teach  %示教

q = [0,0,0,0,0] %机械臂参数初始化
Five_dof.plot(q)

%fkine 正向运动学
q0 = [pi/2 pi/2 0 0 0];
T = Five_dof.fkine(q0)

%fkine 逆向运动学
q1 = Five_dof.ikine(T,'mask',[1 1 1 1 1 0])
q2 = Five_dof.ikunc(T)
%% 机械臂工作空间可视化
% 关节空间随机生成变量--(fkine)-->变换矩阵--(transl)-->三维坐标
% rand() 在[0,1]中随机生成一个数
% 随机关节空间变量 q = qmin+rand(qmax-qmin)
% 定义关节限制Link.qlim
L(1).qlim = [-150,150]/180 * pi;
L(2).qlim = [-100,90] /180 * pi;
L(3).qlim = [-90,90]  /180 * pi;
L(4).qlim = [-100,100]/180 * pi;
L(5).qlim = [-180,180]/180 * pi;

num = 100;
for i = 1:num

    q1 = L(1).qlim(1) + rand*(L(1).qlim(2) - L(1).qlim(1));
    q2 = L(2).qlim(1) + rand*(L(2).qlim(2) - L(2).qlim(1));
    q3 = L(3).qlim(1) + rand*(L(3).qlim(2) - L(3).qlim(1));
    q4 = L(4).qlim(1) + rand*(L(4).qlim(2) - L(4).qlim(1));
    q5 = L(5).qlim(1) + rand*(L(5).qlim(2) - L(5).qlim(1));
    
    q = [q1 q2 q3 q4 q5];

    T = Five_dof.fkine(q);

    P(i,:) = transl(T);
end

plot3(P(:,1),P(:,2),P(:,3),'b.','MarkerSize',1);
%% 轨迹规划
%给定位置：(-580,45,22) --> (75,-460,0) 
T1 = transl(-580,45,22)*troty(180)
T2 = transl(75,-460,0)*troty(180)

q1 = Five_dof.ikunc(T1)
q2 = Five_dof.ikunc(T2)

Five_dof.plot(q1);
pause;
Five_dof.plot(q2);

%% 轨迹绘制
%给定位置：(-580,45,22) --> (75,-460,0) 
P1 = [-45,-538,-20];
P2 = [400,49,200];

t = linspace(0,2,51);
Traj=mtraj(@tpoly,P1,P2,t);

n = size(Traj,1);
T = zeros(4,4,n);

for i = 1:n
    T(:,:, i) = transl(Traj(i,:))*troty(180);
end

Qtraj = Five_dof.ikunc(T);

%Five_dof.plot(Qtraj);  %仅有演示
%Five_dof.plot(Qtraj,'trail','b'); %留下轨迹
Five_dof.plot(Qtraj,'movie','trail.gif'); %留下轨迹
%%
hold on

plot(t,Traj(:,1),'.-','LineWidth',1);
plot(t,Traj(:,2),'.-','LineWidth',1);
plot(t,Traj(:,3),'.-','LineWidth',1);

grid on
legend('x','y','z');
xlabel('time');
ylabel('position')

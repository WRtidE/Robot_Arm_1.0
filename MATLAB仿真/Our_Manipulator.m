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
%% 机械臂工作空间可视化
num = 40000;
for i = 1:num

    q1 = L(1).qlim(1) + rand*(L(1).qlim(2) - L(1).qlim(1));
    q2 = L(2).qlim(1) + rand*(L(2).qlim(2) - L(2).qlim(1));
    q3 = L(3).qlim(1) + rand*(L(3).qlim(2) - L(3).qlim(1));
    q4 = L(4).qlim(1) + rand*(L(4).qlim(2) - L(4).qlim(1));
    q5 = L(5).qlim(1) + rand*(L(5).qlim(2) - L(5).qlim(1));
    
    q = [q1 q2 q3 q4 q5];
    T = Five_dof.fkine(q);
    %Five_dof.plot(q);
    P(i,:) = transl(T);  
end

plot3(P(:,1),P(:,2),P(:,3),'b.','MarkerSize',1);

Five_dof.display();  %查看DH表
Five_dof.teach       %查看GUI
%% 轨迹绘制
%给定位置：(-580,45,22) --> (75,-460,0) 
P1 = [-280,45,212];
P2 = [75,-260,100];

t = linspace(0,2,51);
Traj=mtraj(@tpoly,P1,P2,t);

n = size(Traj,1);
T = zeros(4,4,n);

for i = 1:n
    T(:,:, i) = transl(Traj(i,:))*troty(180);
end

Qtraj = Five_dof.ikunc(T);

Five_dof.plot(Qtraj);  %仅有演示
%Five_dof.plot(Qtraj,'trail','b'); %留下轨迹
%Five_dof.plot(Qtraj,'movie','test.gif'); %留下轨迹

%% 绘制轨迹变化曲线
hold on

plot(t,Traj(:,1),'.-','LineWidth',1);
plot(t,Traj(:,2),'.-','LineWidth',1);
plot(t,Traj(:,3),'.-','LineWidth',1);

grid on
legend('x','y','z');
xlabel('time');
ylabel('position')
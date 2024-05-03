%% OUR_Manipulator
clear;
clc;
% DH表
theta1 = 0; D1 = 250; A1 = 0;    alpha1 = pi/2;  offset1 = pi/2;
theta2 = 0; D2 = 0;   A2 = 250;  alpha2 = 0;     offset2 = pi/2;
theta3 = 0; D3 = 0;   A3 = 250;  alpha3 = 0;     offset3 = pi/2;
theta4 = 0; D4 = -45; A4 = 116;  alpha4 = -pi/2; offset4 = 0;
theta5 = 0; D5 = 250; A5 = 0;    alpha5 = 0;     offset5 = 0;
% 创建关节
L(1) = Link('revolute','d',D1,'a',A1,'alpha', alpha1,'offset', offset1);
L(2) = Link('revolute','d',D2,'a',A2,'alpha', alpha2,'offset', offset2);
L(3) = Link('revolute','d',D3,'a',A3,'alpha', alpha3,'offset', offset3);
L(4) = Link('revolute','d',D4,'a',A4,'alpha', alpha4,'offset', offset4);
L(5) = Link('revolute','d',D5,'a',A5,'alpha', alpha5,'offset', offset5);

Five_dof=SerialLink(L,'name','5-dof');
Five_dof.base = transl(0,0,0);

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
%% 关节空间可视化
num = 30000;
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
Five_dof.teach   
%% Joint下路径规划
%定点
P1 = [-45,-538,-20];
via_points = [400,49,200];
P2 = [400,49,0];

T1 = transl(P1)*troty(180)
T2 = transl(P2)*troty(180)

q1 = Five_dof.ikunc(T1)
q2 = Five_dof.ikunc(T2)



Five_dof.plot(q1);
pause;
Five_dof.plot(q2);

%
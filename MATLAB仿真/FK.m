function [T0_tool] = FK(Angle_T,a,d,alpha,l)
syms thetas as ds al
%传入角度
theta1 = Angle_T(1, 1);
theta2 = Angle_T(1, 2);
theta3 = Angle_T(1, 3);
theta4 = Angle_T(1, 4);
theta5 = Angle_T(1, 5);

d1 = d(1); d2 = d(2); d3 = d(3); d4 = d(4); d5 = d(5); 
a1 = a(1); a2 = a(2); a3 = a(3); a4 = a(4); a5 = a(5); 
alpha1 = alpha(1); alpha2 = alpha(2); alpha3 = alpha(3); 
alpha4 = alpha(4); alpha5 = alpha(5); 


Ti_1i = [      cos(thetas)           -sin(thetas)           0              as;
       sin(thetas)*cos(al)    cos(thetas)*cos(al)    -sin(al)     -sin(al)*ds;
       sin(thetas)*sin(al)    cos(thetas)*sin(al)     cos(al)      cos(al)*ds;
                         0                     0            0               1];
% 需要修改偏置
%subs将Ti_1i中的变量[thetas ...]替换成[theta1 ...]
%double转为双精度浮点数
T01 = subs(Ti_1i, [thetas al as ds],[theta1 alpha1 a1 d1]);
T12 = subs(Ti_1i, [thetas al as ds],[theta2+pi/2 alpha2 a2 d2]);
T23 = subs(Ti_1i, [thetas al as ds],[theta3+pi/2 alpha3 a3 d3]);
T34 = subs(Ti_1i, [thetas al as ds],[theta4-pi/2 alpha4 a4 d4]);
T45 = double(subs(Ti_1i, [thetas al as ds],[theta5 alpha5 a5 d5]));
T5_tool = transl(0,0,l); %执行器tool的位移

T0_tool = double(T01*T12*T23*T34*T45*T5_tool); %从基坐标系到工具坐标系
end

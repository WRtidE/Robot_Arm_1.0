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
% % 绘制motor1路径曲线
% subplot(3,2,1)
% ylabel('motor1')
% plot(t,motor1.theta,'r','linewidth',2)
% hold on;
% plot(t,motor1.vel,'g','linewidth',2)
% hold on;
% plot(t,motor1.acc,'b','linewidth',2)
% grid on;
% hold on;
% 
% % 绘制motor2路径曲线
% subplot(3,2,2)
% ylabel('motor2')
% plot(t,motor2.theta,'r','linewidth',2)
% hold on;
% plot(t,motor2.vel,'g','linewidth',2)
% hold on;
% plot(t,motor2.acc,'b','linewidth',2)
% grid on;
% hold on;
% 
% 
% % 绘制motor3路径曲线
% subplot(3,2,3)
% ylabel('motor3')
% plot(t,motor3.theta,'r','linewidth',2)
% hold on;
% plot(t,motor3.vel,'g','linewidth',2)
% hold on;
% plot(t,motor3.acc,'b','linewidth',2)
% grid on;
% hold on;
% 
% % 绘制motor4路径曲线
% subplot(3,2,4)
% ylabel('motor4')
% plot(t,motor4.theta,'r','linewidth',2)
% hold on;
% plot(t,motor4.vel,'g','linewidth',2)
% hold on;
% plot(t,motor4.acc,'b','linewidth',2)
% grid on;
% hold on;
% 
% % 绘制motor5路径曲线
% subplot(3,2,5)
% ylabel('motor5')
% plot(t,motor5.theta,'r','linewidth',2)
% hold on;
% plot(t,motor5.vel,'g','linewidth',2)
% hold on;
% plot(t,motor5.acc,'b','linewidth',2)
% grid on;
% hold on;

%传回结构体
res.motor1 = motor1;
res.motor2 = motor2;
res.motor3 = motor3;
res.motor4 = motor4;
res.motor5 = motor5;

path = res;

end



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


motor.theta = theta;
motor.vel   = vel;
motor.acc   = acc;

res = motor;
end


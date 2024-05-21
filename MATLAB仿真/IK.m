function  theta = IK(T0_tool,a,d,l)
    
    % 机械臂坐标系到机器人执行器坐标系的矩阵
    r11 = T0_tool(1, 1); r21 = T0_tool(2, 1); r31 = T0_tool(3, 1);
    r12 = T0_tool(1, 2); r22 = T0_tool(2, 2); r32 = T0_tool(3, 2);
    r13 = T0_tool(1, 3); r23 = T0_tool(2, 3); r33 = T0_tool(3, 3);
    px =  T0_tool(1, 4); py =  T0_tool(2, 4); pz =  T0_tool(3, 4);
    
    % 机械臂参数
    d1 = d(1); d2 = d(2); d3 = d(3); d4 = d(4); d5 = d(5); 
    a1 = a(1); a2 = a(2); a3 = a(3); a4 = a(4); a5 = a(5);  
   

    %% theta1 两个解
    theta1 = zeros(1,2);

    theta1(1,1) = antiSinCos(py,px);
    theta1(1,2) = antiSinCos(-py,-px);

    
    %% theta5 
    theta5 = zeros(1,2);
    theta5(1,1) = antiSinCos(r21*cos(theta1(1,1))-r11*sin(theta1(1,1)),r22*cos(theta1(1,1))-r12*sin(theta1(1,1)));
    theta5(2,1) = antiSinCos(r21*cos(theta1(1,2))-r11*sin(theta1(1,2)),r22*cos(theta1(1,2))-r12*sin(theta1(1,2)));
    sins = r21*cos(theta1(1,1))-r11*sin(theta1(1,1))
    coss = r22*cos(theta1(1,1))-r12*sin(theta1(1,1))
    theta5

    % 储存数据：生成一个[theta5(1,1),theta5(1,2),theta5(1,1),theta5(2,2)]的行
    theta5_use = [theta5(1,1),theta5(2,1)];
    theta5 = repmat(theta5_use,1,2);

    %% theta3
    theta3 = zeros(2,2);
    for i =1:2
        x = px*cos(theta1(i)) - d5*(r13*cos(theta1(i)) + r23*sin(theta1(i))) + py*sin(theta1(i));
        y = pz - d1 - d5*r33;
        
        te = (x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4)
        %如果abs(sin)的值大于1,立即推放弃这个解；其余情况剩下两个解
        if (x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4)>1 || (x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4)<-1
            theta3(1,i) = inf;
            theta3(2,i) = inf;
        else
            theta3(1,i) = asin(-(x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4));
            theta3(2,i) = pi- asin(-(x.^2+y.^2-a4.^2-a3.^2)/(2*a3*a4)) ;
        end
     
    end

    %排成一行
    theta3 = [theta3(1,:),theta3(2,:)]
   
    %越界判断
    for i =1:4
        if theta3(i)>3.14
            theta3(i) = theta3(i) - 2*pi;
        elseif theta3(i)<-3.14
            theta3(i) = theta3(i) + 2*pi;
        end
    end
    %% theta2
    theta2 = zeros(1,4);
    theta1 = reshape(repmat(theta1,1,2),1,[]);

    for i =1:4  
        x = px*cos(theta1(i)) - d5*(r13*cos(theta1(i)) + r23*sin(theta1(i))) + py*sin(theta1(i));
        y = pz - d1 - d5*r33;

        r = [-a4*cos(theta3(i))       a4*sin(theta3(i))-a3;
             -a4*sin(theta3(i))+a3   -a4*cos(theta3(i))];
        if theta3(i) < 10^10 % 判断theta3是否有解
              %解方程
              %sc2  = inv(r)*[x;y]
              %sc2(1) = - (a3*d1 - a3*pz - a4*d1*sin(theta3(i)) + a4*pz*sin(theta3(i)) + a3*d5*r33 - a4*d5*r33*sin(theta3(i)) + a4*px*cos(theta1(i))*cos(theta3(i)) + a4*py*cos(theta3(i))*sin(theta1(i)) - a4*d5*r13*cos(theta1(i))*cos(theta3(i)) - a4*d5*r23*cos(theta3(i))*sin(theta3(i)))/(a3^2 - 2*sin(theta3(i))*a3*a4 + a4^2);
              %sc2(2) = (a4*cos(theta3(i))*(d1 - pz + d5*r33))/(a3^2 - 2*sin(theta3(i))*a3*a4 + a4^2) - ((a3 - a4*sin(theta3(i)))*(px*cos(theta1(i)) + py*sin(theta3(i)) - d5*r13*cos(theta1(i)) - d5*r23*sin(theta1(i))))/(a3^2 - 2*sin(theta3(i))*a3*a4 + a4^2);

              sc2(1) =  - ((a3 - a4*sin(theta3(i)))*(d1 - pz + d5*r33))/(a4^2*cos(theta3(i))^2 + a4^2*sin(theta3(i))^2 + a3^2 - 2*a3*a4*sin(theta3(i))) - (a4*cos(theta3(i))*(px*cos(theta1(i)) - d5*(r13*cos(theta1(i)) + r23*sin(theta1(i))) + py*sin(theta1(i))))/(a4^2*cos(theta3(i))^2 + a4^2*sin(theta3(i))^2 + a3^2 - 2*a3*a4*sin(theta3(i)))
              sc2(2) =    (a4*cos(theta3(i))*(d1 - pz + d5*r33))/(a4^2*cos(theta3(i))^2 + a4^2*sin(theta3(i))^2 + a3^2 - 2*a3*a4*sin(theta3(i))) - ((a3 - a4*sin(theta3(i)))*(px*cos(theta1(i)) - d5*(r13*cos(theta1(i)) + r23*sin(theta1(i))) + py*sin(theta1(i))))/(a4^2*cos(theta3(i))^2 + a4^2*sin(theta3(i))^2 + a3^2 - 2*a3*a4*sin(theta3(i)))

              
              if sc2(1)>1 || sc2(2)>1 || sc2(1)<-1 || sc2(2)<-1
                theta2(i) = inf;
            else
                theta2(i) = antiSinCos(sc2(2),sc2(1));
            end
        else %theta3无解则theta2也无解
            theta2(i) = inf;
        end
    end

    %越界判断
    for i =1:4
        if theta2(i)>3.14
            theta2(i) = theta2(i) - 2*pi;
        elseif theta2(i)<-3.14
            theta2(i) = theta2(i) + 2*pi;
        end
    end

    %% theta4
    theta(1,:) = theta1;
    theta(2,:) = theta2;
    theta(3,:) = theta3;
    theta(5,:) = theta5;

    for i =1:4
        c = r31*cos(theta(5,i)) - r32*sin(theta(5,i))
        s = r11*cos(theta(1,i))*cos(theta(5,i)) - r12*cos(theta(1,i))*sin(theta(5,i)) + r21*cos(theta(5,i))*sin(theta(1,i)) - r22*sin(theta(1,i))*sin(theta(5,i))
        theta(4,i) = antiSinCos(-s,c)-theta(2,i)-theta(3,i);
        
        % 越界判断
        if theta(4,i)>3.14
            theta(4,i) = theta(4,i) - 2*pi;
        elseif theta(4,i)<-3.14
            theta(4,i) = theta(4,i) + 2*pi;
        end
    end

    theta = theta';

end
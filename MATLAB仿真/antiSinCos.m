%用来求arctan的函数
function angle = antiSinCos(sA,cA)
    eps = 1e-8;
    angle = 0;
    
    %abs(cos) = 0 和abs(sin) = 0 的时候，让theta = 0；
    if(abs(sA) < eps) && (abs(cA) < eps)
        return ;
    end
    
    %abs(cos) = 0 的时候，让theta = 90 *（sin的符号）
    if abs(cA) < eps   
        angle = pi/2.0*sign(sA);

    %abs(cos = 0 的时候，让theta = 0     
    elseif abs(sA) < eps
%         if sign(cA) == 1
%             return ;
%         else
%             angle = pi;
%             return ;
%         end   ​
    else
        angle = atan2(sA, cA);
    end

end
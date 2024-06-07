%将输入的x，y，z坐标转换为FK矩阵
function T_target = target_calc(x,y,z)

 %物体相对于机械臂底座的位置
 T_objection = [1 0 0 x; 0 1 0 y; 0 0 1 0; 0 0 0 1];
 %执行器相对于机械臂末端的矩阵
 T_tool      = [0 1 0 0; 1 0 0 0; 0 0 -1 z+63; 0 0 0 1];

 T_target = T_objection * T_tool;
end
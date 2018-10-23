function [next_vec, next_u, massion ] = main_pf( u,x, pic )
%人工势场法壁障规划入口函数，调用人工势场法
%控制调用规划，规划接受当前位姿，结合感知的信息（仿真的略去感知模型），给出期望信息
global record record_x t h T

obj_r = 15;%障碍物实际半径
goal = [400,400,0];%Goal Point
obj(1,:) = [100,100,0];%%球形障碍物位置，z为正表示在水下
obj(2,:) = [200,200,0];
obj(3,:) = [300,275,0];
obj(4,:) = [300,325,0];
obj_num = length(obj);
target = sqrt((goal(3) - x(3))^2+(goal(2) - x(2))^2+(goal(1) - x(1))^2);
if (target < 5)||(t>T-2*h)%把这个半径10设置小一点挺好，然后就是调整参数优化了
    massion = 1;%0未到达，1已经到达
else
    massion = 0;
end

dis = zeros(1,obj_num);

[next_vec, record(round(t/h),1:obj_num+1)] = potential_field(u,x,goal,obj,obj_num,dis,t);
next_u =2;
record_x(round(t/h),1:length(x)) = x;%记录AUV轨迹
% 如果完成任务，控制在终止前发来绘图命令，进行绘图，然后结束仿真。
record_x(round(t/h),length(x)+1) = target;
if pic == 1
    
    figure;
    for obj_i=1:obj_num
        hold on, [obj_x,obj_y,obj_z]=ellipsoid(obj(obj_i,1),obj(obj_i,2),obj(obj_i,3),obj_r,obj_r,obj_r);
        surf(obj_x,obj_y,obj_z);
    end
    hold on,plot3(record_x(:,1),record_x(:,2),record_x(:,3),'k.');%AUV轨迹
    title('轨迹曲线');xlabel('X');ylabel('Y');zlabel('Z');
    hold on, plot3(record_x(1,1),record_x(1,2),record_x(1,3),'x','color','green','LineWidth',3);%起点
    hold on, plot3(goal(1),goal(2),goal(3),'x','color','red','LineWidth',3);%终点
    view(3);
    
    figure;
    for obj_i = 1:obj_num
        hold on, plot(record(:,obj_num+1),record(:,obj_i),'LineWidth',2);
    end

end

end


function [next_vec, dis] = potential_field(u,x,goal,obj,obj_num,dis,t)
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明

goal_k = 50;
% obj_eta = 5e10;
obj_eta = 5e9;%根据实际进行调整每次10倍这样调整比较好
% obj_real_r = 15;%障碍物实际半径
obj_r =30;%障碍物影响半径，可观测到就开始躲避,提升影响半径，减小运动轨迹抖动。
%但是路径不会优，取实际的一般就能有更短的路径。
%多障碍团成一团的时候，可能要取大一点的影响半径
obj_vec = zeros(obj_num,3);
%     x(3) = -x(3);
    %势场模型
    %斥力模型
    %距离(X，X_obj)
    for obj_i=1:obj_num
        
        obj_l = [x(1)-obj(obj_i,1),x(2)-obj(obj_i,2),x(3)-obj(obj_i,3)];%障碍物到目标矢量
        dis(obj_i) = norm(obj_l);
        %将障碍物转换到AUV坐标系，AUV从路径短的一边走
        u_vec = [u(1),u(2),u(3)];
        obj_cross = cross(u_vec./norm(u_vec),obj_l./norm(obj_l));
        obj_cross = cross(obj_l./norm(obj_l),obj_cross);%对于共线×乘为0，任取一侧作为
        
        if norm(obj_l) < obj_r
            obj_f = 0.5*obj_eta*(1/norm(obj_l)-1/obj_r)*((1/norm(obj_l))^2);%斥力模型,漩涡型斥力场
            obj_vec(obj_i,:) = obj_f.*obj_cross/norm(obj_cross);
        else
            obj_vec(obj_i,:) = 0;
        end
        %AUV到障碍物量单位化
    end
    obj_f_sum = sum(obj_vec);
    
    %引力模型
    goal_l = -[x(1)-goal(1),x(2)-goal(2),x(3)-goal(3)];
%     goal_l_0 = -[x_0(1)-point(1),x_0(2)-point(2),x_0(3)-point(3)];
    goal_f = goal_k*norm(goal_l);
    goal_vec = goal_f.*goal_l./norm(goal_l);%多此一举
    
    %克服局部极小点

    %合力矢量
    next_vec = goal_vec+obj_f_sum;
    dis(obj_num+1) = t;%让最后记录时间
end


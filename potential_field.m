function [next_vec, dis] = potential_field(u,x,goal,obj,obj_num,dis,t)
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��

goal_k = 50;
% obj_eta = 5e10;
obj_eta = 5e9;%����ʵ�ʽ��е���ÿ��10�����������ȽϺ�
% obj_real_r = 15;%�ϰ���ʵ�ʰ뾶
obj_r =30;%�ϰ���Ӱ��뾶���ɹ۲⵽�Ϳ�ʼ���,����Ӱ��뾶����С�˶��켣������
%����·�������ţ�ȡʵ�ʵ�һ������и��̵�·����
%���ϰ��ų�һ�ŵ�ʱ�򣬿���Ҫȡ��һ���Ӱ��뾶
obj_vec = zeros(obj_num,3);
%     x(3) = -x(3);
    %�Ƴ�ģ��
    %����ģ��
    %����(X��X_obj)
    for obj_i=1:obj_num
        
        obj_l = [x(1)-obj(obj_i,1),x(2)-obj(obj_i,2),x(3)-obj(obj_i,3)];%�ϰ��ﵽĿ��ʸ��
        dis(obj_i) = norm(obj_l);
        %���ϰ���ת����AUV����ϵ��AUV��·���̵�һ����
        u_vec = [u(1),u(2),u(3)];
        obj_cross = cross(u_vec./norm(u_vec),obj_l./norm(obj_l));
        obj_cross = cross(obj_l./norm(obj_l),obj_cross);%���ڹ��ߡ���Ϊ0����ȡһ����Ϊ
        
        if norm(obj_l) < obj_r
            obj_f = 0.5*obj_eta*(1/norm(obj_l)-1/obj_r)*((1/norm(obj_l))^2);%����ģ��,�����ͳ�����
            obj_vec(obj_i,:) = obj_f.*obj_cross/norm(obj_cross);
        else
            obj_vec(obj_i,:) = 0;
        end
        %AUV���ϰ�������λ��
    end
    obj_f_sum = sum(obj_vec);
    
    %����ģ��
    goal_l = -[x(1)-goal(1),x(2)-goal(2),x(3)-goal(3)];
%     goal_l_0 = -[x_0(1)-point(1),x_0(2)-point(2),x_0(3)-point(3)];
    goal_f = goal_k*norm(goal_l);
    goal_vec = goal_f.*goal_l./norm(goal_l);%���һ��
    
    %�˷��ֲ���С��

    %����ʸ��
    next_vec = goal_vec+obj_f_sum;
    dis(obj_num+1) = t;%������¼ʱ��
end


function [next_vec, next_u, massion ] = main_pf( u,x, pic )
%�˹��Ƴ������Ϲ滮��ں����������˹��Ƴ���
%���Ƶ��ù滮���滮���ܵ�ǰλ�ˣ���ϸ�֪����Ϣ���������ȥ��֪ģ�ͣ�������������Ϣ
global record record_x t h T

obj_r = 15;%�ϰ���ʵ�ʰ뾶
goal = [400,400,0];%Goal Point
obj(1,:) = [100,100,0];%%�����ϰ���λ�ã�zΪ����ʾ��ˮ��
obj(2,:) = [200,200,0];
obj(3,:) = [300,275,0];
obj(4,:) = [300,325,0];
obj_num = length(obj);
target = sqrt((goal(3) - x(3))^2+(goal(2) - x(2))^2+(goal(1) - x(1))^2);
if (target < 5)||(t>T-2*h)%������뾶10����Сһ��ͦ�ã�Ȼ����ǵ��������Ż���
    massion = 1;%0δ���1�Ѿ�����
else
    massion = 0;
end

dis = zeros(1,obj_num);

[next_vec, record(round(t/h),1:obj_num+1)] = potential_field(u,x,goal,obj,obj_num,dis,t);
next_u =2;
record_x(round(t/h),1:length(x)) = x;%��¼AUV�켣
% ���������񣬿�������ֹǰ������ͼ������л�ͼ��Ȼ��������档
record_x(round(t/h),length(x)+1) = target;
if pic == 1
    
    figure;
    for obj_i=1:obj_num
        hold on, [obj_x,obj_y,obj_z]=ellipsoid(obj(obj_i,1),obj(obj_i,2),obj(obj_i,3),obj_r,obj_r,obj_r);
        surf(obj_x,obj_y,obj_z);
    end
    hold on,plot3(record_x(:,1),record_x(:,2),record_x(:,3),'k.');%AUV�켣
    title('�켣����');xlabel('X');ylabel('Y');zlabel('Z');
    hold on, plot3(record_x(1,1),record_x(1,2),record_x(1,3),'x','color','green','LineWidth',3);%���
    hold on, plot3(goal(1),goal(2),goal(3),'x','color','red','LineWidth',3);%�յ�
    view(3);
    
    figure;
    for obj_i = 1:obj_num
        hold on, plot(record(:,obj_num+1),record(:,obj_i),'LineWidth',2);
    end

end

end


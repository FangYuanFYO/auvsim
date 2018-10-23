%����������ϵͳ������S�������/PD�������������ɶȿ��ƣ����ƽ���ʮ����β�档
%ʹ�ã�ͨ���������滮��������ɸ��ַ��档

%����ֻ�г�ʼλ�ú͵�ǰλ����Ϣ�����������ܡ�

%   delta�����ǽǶ�,delta_sβ������1��delta_b��������2,delta_r��β�����3,����Ƕ�ֵ
%   theta�����--q,PHI--p,PSI--r����Ϊ����ֵ������ת����ʱ
%   ʡȥ���������������Ժ���������������Ҫ��һ�㣬F=ma,��Ȼ���ٶȺ�С��
%�ռ��˶�����,������һ�������У�������ٶȣ����أ�Ȼ����ã��ݹ麯��
%д��6*1ϵ����C[i]

close all
clear
global t h T;
f=zeros(6,1);%�ƽ���
u=zeros(6,1);%��ʼ�ٶ�
x=zeros(6,1);%��ʼλ��
t=0.00;%��ǰ����ʱ��
h=0.01;%���沽��
T=500;%�������ǰ���ʱ��

delta=[0,0,0];%��ʼ��ǣ�����ǣ����������ת��������Ǳʵ��delta=[5,0,5]��
% file=fopen('Data.txt','w+');
% fprintf(file,'t-------X-------Y-------Z-------phi-------theta-------psi\r\n');
% fclose(file);
pic = 0;
mode = input('1-main_pf;  2-intertracking\n');
c = [1 1 0.1];
%�Ƴ�������
while(t<=T)
    t=t+h;
    k1=uuv_maneuve(u,x,delta,f);
    k2=uuv_maneuve(u+0.5*h*k1,x,delta,f);
    k3=uuv_maneuve(u+0.5*h*k2,x,delta,f);
    k4=uuv_maneuve(u+h*k3,x,delta,f);
    u=u+(h/6)*(k1+2*k2+2*k3+k4);
    
    nu=uuv_b2n(u,x);%��ϵ���ٶȺ�delta_x��delta_theta
    x=nu*h+x;
    
    switch(mode)
        case 1
            [next_vec, next_u, massion ] = main_pf( u,x, pic );
            if massion == 1
                pic = 1;
                [next_vec, next_u, massion ] = main_pf( u,x, pic );%�˹��Ƴ��������
                break;
            end
        case 2
            [vec, next_vec, next_u, massion ] = intertracking( x,u,delta, pic );
            if massion == 1
                pic = 1;
                [vec, next_vec, next_u, massion ] = intertracking( x,u,delta, pic );%�켣����
                break;
            end
    end
    
    b_u=next_u;
    b_angle_z = atan2(next_vec(2), next_vec(1));
    b_angle_y = -atan2(next_vec(3), norm(next_vec(1:3)));
    alpha = -atan2(u(3),u(1));
    beta = atan2(u(2),u(1));
    sum = norm(vec(1:3));
    e_dx(1) = b_u - norm(u(1:3)) + c(1)*(vec(1)-x(1))/sum;

    e_dx(2) = b_angle_y - (x(5)+alpha) - c(2)*(vec(3)-x(3))/sum;%theta
    
    e_dx(3) = b_angle_z - (x(6)+beta) + c(3)*(vec(2)-x(2))/sum;%psi

    while  e_dx(2) > pi
        e_dx(2) =  e_dx(2) - 2*pi;
    end
    while  e_dx(2) < -pi
        e_dx(2) =  e_dx(2) + 2*pi;
    end
    while  e_dx(3) > pi
        e_dx(3) = e_dx(3) -2*pi;
    end
    while e_dx(3) < -pi
        e_dx(3) = e_dx(3) + 2*pi;
    end
    
    e_dxdt = e_dx/h;

    [sys] = three_uuv_Ssurface(e_dx,e_dxdt);
    f(1) = sys(1);
    delta(1) = sys(2);%r
    delta(3) = sys(3);%p,delta_b;delta(2) = -sys(3)%ֹͣʹ��;delta(3) = sys(3)
    
end

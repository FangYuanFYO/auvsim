%描述：控制系统，调用S面控制器/PD控制器，六自由度控制，单推进器十字形尾舵。
%使用：通过和其他规划的连接完成各种仿真。

%控制只有初始位置和当前位置信息。其他均不管。

%   delta输入舵角角度,delta_s尾升降舵1和delta_b首升降舵2,delta_r是尾方向舵3,输入角度值
%   theta纵倾角--q,PHI--p,PSI--r，均为弧度值，坐标转换中时
%   省去重力项，简化推力项，忽略海流作用力推力需要大一点，F=ma,不然加速度很小。
%空间运动方程,放入另一个函数中，计算出速度，返回，然后调用，递归函数
%写出6*1系数阵C[i]

close all
clear
global t h T;
f=zeros(6,1);%推进器
u=zeros(6,1);%初始速度
x=zeros(6,1);%初始位置
t=0.00;%当前仿真时间
h=0.01;%仿真步长
T=500;%完成任务前的最长时间

delta=[0,0,0];%初始舵角；首向角（正的向左回转）螺旋下潜实验delta=[5,0,5]。
% file=fopen('Data.txt','w+');
% fprintf(file,'t-------X-------Y-------Z-------phi-------theta-------psi\r\n');
% fclose(file);
pic = 0;
mode = input('1-main_pf;  2-intertracking\n');
c = [1 1 0.1];
%势场法避障
while(t<=T)
    t=t+h;
    k1=uuv_maneuve(u,x,delta,f);
    k2=uuv_maneuve(u+0.5*h*k1,x,delta,f);
    k3=uuv_maneuve(u+0.5*h*k2,x,delta,f);
    k4=uuv_maneuve(u+h*k3,x,delta,f);
    u=u+(h/6)*(k1+2*k2+2*k3+k4);
    
    nu=uuv_b2n(u,x);%定系的速度和delta_x，delta_theta
    x=nu*h+x;
    
    switch(mode)
        case 1
            [next_vec, next_u, massion ] = main_pf( u,x, pic );
            if massion == 1
                pic = 1;
                [next_vec, next_u, massion ] = main_pf( u,x, pic );%人工势场壁障入口
                break;
            end
        case 2
            [vec, next_vec, next_u, massion ] = intertracking( x,u,delta, pic );
            if massion == 1
                pic = 1;
                [vec, next_vec, next_u, massion ] = intertracking( x,u,delta, pic );%轨迹跟踪
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
    delta(3) = sys(3);%p,delta_b;delta(2) = -sys(3)%停止使用;delta(3) = sys(3)
    
end

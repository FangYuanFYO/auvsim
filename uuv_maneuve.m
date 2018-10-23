function [sys]=uuv_maneuve(u,x,delta,f)

phi=x(4);%--p
theta=x(5);%--q
psi=x(6);%--r
delta_r=delta(1);
delta_s=delta(2);
delta_b=delta(3);
%七个输入量分别是驱动力主推，三个初始姿态角，三个初始舵角

u_=u(1);
v=u(2);
w=u(3);
p=u(4);
q=u(5);
r=u(6);

p_water=1000;%水密度
L=70;%长
D=6;%宽
m=1500000;%质量
g=9.81;% 重力加速度
%重心位置:
xg=0;
yg=0;
zg=0;
xb=0;
yb=0;
zb=-0.5;

%惯量：
Ix=4750222.167;
Iy=325835635.157;
Iz=325835779.769;

%水动力系数：
x_u_=-1.39e-4;
y_r_=3.02e-5;
y_v_=-7.76e-3;
y_r=4.59e-3;
y_v=-1.52e-2;
y_delta_r=-3.76e-3;
z_q_=-4.42e-5;
z_w_=-7.51e-3;
z_q=-4.13e-3;
z_w=-1.32e-2;
z_delta_s=-3.76e-3;
z_delta_b=-2.09e-3;
k_vq=-1.4000e-05;
k_wr=1.4000e-05;
m_q_=-3.65e-4;
m_w_=-4.42e-5;
m_q=-2.52e-3;
m_w=4.25e-3;
m_delta_s=-1.72e-3;
m_delta_b=7.46e-4;
n_r_=-3.65e-4;
n_v_=3.02e-5;
n_r=-2.45e-3;
n_v=-3.79e-3;
n_delta_r=1.72e-3;

M=[m-0.5*p_water*(L^3)*x_u_, 0, 0, 0, 0, 0;
    0, m-0.5*p_water*(L^3)*y_v_, 0, 0, 0, -0.5*p_water*(L^4)*y_r_;
    0, 0, m-0.5*p_water*(L^3)*z_w_, 0, -0.5*p_water*(L^4)*z_q_, 0;
    0, 0, 0, Ix, 0, 0;
    0, 0, -0.5*p_water*(L^4)*m_w_, 0, Iy-0.5*p_water*(L^5)*m_q_, 0;
    0, -0.5*p_water*(L^4)*n_v_, 0, 0, 0, Iz-0.5*p_water*(L^5)*n_r_];%惯量矩阵6*6

% V=[0, 0, 0, 0, -m*w, m*v;
%     0.5*p_water*(L^3)*y_r*r+0.5*p_water*(L^3)*y_v*v, 0, 0, m*w, 0, -m*u_;
%     0.5*p_water*(L^3)*z_q*q+0.5*p_water*(L^3)*z_w*w, 0, 0, -m*v, m*u_, 0;
%     0, 0.5*p_water*(L^4)*k_vq*q, 0.5*p_water*(L^4)*k_wr*r, 0, -Iz*r, Iy*q;
%     0.5*p_water*(L^4)*m_q*q+0.5*p_water*(L^3)*m_w*w, 0, 0, Iz*r,0,-Ix*p;
%     0.5*p_water*(L^4)*n_r*r+0.5*p_water*(L^4)*n_v*v, 0, 0, -Iy*q, Ix*p, 0];
% 哥氏力矩阵6*6,从六自由度公式推出来的和下面那个的离心力部分符号相反

 C=[0, 0, 0, 0, m*w, -m*v;
     0.5*p_water*(L^3)*y_r*r+0.5*p_water*(L^3)*y_v*v, 0, 0, -m*w, 0, m*u_;
     0.5*p_water*(L^3)*z_q*q+0.5*p_water*(L^3)*z_w*w, 0, 0, m*v, -m*u_, 0;
     0, 0.5*p_water*(L^4)*k_vq*q, 0.5*p_water*(L^4)*k_wr*r, 0, Iz*r, -Iy*q;
     0.5*p_water*(L^4)*m_q*q+0.5*p_water*(L^3)*m_w*w, 0, 0, -Iz*r, 0, Ix*p;
     0.5*p_water*(L^4)*n_r*r+0.5*p_water*(L^4)*n_v*v, 0, 0, Iy*q, -Ix*p, 0];
% %哥氏力矩阵6*6,这个的水平回转运动仿真效果特别好,比上一个好，但是控制的震荡要厉害一点。

G=[ 0;
    0;
    0;
    -m*g*(zg-zb)*cos(theta)*sin(phi);
    -m*g*(zg-zb)*sin(theta);
    0];%重力
tau=f;%驱动推力
rudder=[0;
    0.5*p_water*(L^2)*y_delta_r*(u_^2)*delta_r;
    0.5*p_water*(L^2)*z_delta_s*(u_^2)*delta_s+0.5*p_water*(L^2)*z_delta_b*(u_^2)*delta_b;
    0;
    0.5*p_water*(L^3)*m_delta_s*(u_^2)*delta_s+0.5*p_water*(L^3)*m_delta_b*(u_^2)*delta_b;
    0.5*p_water*(L^3)*n_delta_r*(u_^2)*delta_r];%舵力

sys = M\(C*u+rudder+tau+G);%操纵性方程inv(M)*(C+V+rudder+tau+G)
end
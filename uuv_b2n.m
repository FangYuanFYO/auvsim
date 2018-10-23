function [ sys ] = uuv_b2n( u,x )
%u-dx
%输入三个姿态角和六个自由度的{B}速度，输出六个速度{N}
sys=zeros(6,1);
phi=x(4);%--p
theta=x(5);%--q
psi=x(6);%--r

% u_=u(4);
% v=u(5);
% w=u(6);
% p=u(7);
% q=u(8);
% r=u(9);

inv_T1=[cos(psi)*cos(theta),sin(psi)*cos(theta),-sin(theta);
    cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi),cos(theta)*sin(phi);
    cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi),sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),cos(theta)*cos(phi)];
inv_pqr=[1,sin(phi)*tan(theta),cos(phi)*tan(theta);0,cos(phi),-sin(phi);0,sin(phi)*sec(theta),cos(phi)*sec(theta)];

nu=(inv_T1')*[u(1);u(2);u(3)];
nb=inv_pqr*[u(4);u(5);u(6)];

sys(1) = nu(1);%速度{n}u
sys(2) = nu(2);%速度{n}v
sys(3) = nu(3);%速度{n}w,全局坐标系Z轴向上,这样auv全局坐标和障碍物一样，方向适量不会；nu(3)为正，Z朝下;可绕x转90deg变到正常的大地坐标
sys(4) = nb(1);%速度{n}p
sys(5) = nb(2);%速度{n}q
sys(6) = nb(3);%速度{n}r

end


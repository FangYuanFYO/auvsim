function [ vec, next_vec, next_u, massion ] = intertracking( x,u,delta,pic )
%三维位置分量，速度分量，任务状态
%曲线轨迹跟踪测试
global t h T rcd_trc rcd_trc_x rcd_trc_u rcd_trc_delta rcd_trc_x_e rcd_trc_next_u
vec = [t, 200*cos((pi/200)*t)-200, 0.1*t];%期望轨迹

next_vec = [1, -pi*sin((pi/200)*t), 0.1];

next_u = norm(next_vec(1:3));

rcd_trc(round(t/h),1:3) = vec;
rcd_trc_x(round(t/h),1:3) = x(1:3);
rcd_trc_x_e(round(t/h),1:3) = [vec(1) - x(1), vec(2) - x(2), vec(3) - x(3)];
rcd_trc_u(round(t/h)) = u(1);
rcd_trc_next_u(round(t/h)) = next_u;
rcd_trc_delta(round(t/h),1:3) = delta;
if pic==1
   figure
   subplot(2,2,1);
   plot3(rcd_trc(:,1),rcd_trc(:,2),rcd_trc(:,3),'.-','LineWidth',2);
   hold on, plot3(rcd_trc_x(:,1),rcd_trc_x(:,2),rcd_trc_x(:,3),'LineWidth',2);
   
   subplot(2,2,2);
   
   plot(rcd_trc(:,1),rcd_trc(:,2));
   hold on, plot(rcd_trc_x(:,1),rcd_trc_x(:,2));
   
   subplot(2,2,3);
   plot(rcd_trc(:,2),rcd_trc(:,3));
   hold on, plot(rcd_trc_x(:,2),rcd_trc_x(:,3));
   
   subplot(2,2,4);
   plot(rcd_trc(:,1),rcd_trc(:,3));
   hold on, plot(rcd_trc_x(:,1),rcd_trc_x(:,3));
   
   figure
   subplot(3,1,1);
   plot(rcd_trc(:,1),rcd_trc_x_e(:,1));
   subplot(3,1,2);
   plot(rcd_trc(:,1),rcd_trc_x_e(:,2));
   subplot(3,1,3);
   plot(rcd_trc(:,1),rcd_trc_x_e(:,3));
   
   figure
   subplot(2,1,1);
   plot(rcd_trc(:,1),rcd_trc_delta(:,1));
   subplot(2,1,2);
   plot(rcd_trc(:,1),rcd_trc_delta(:,3));
   
   figure
   plot(rcd_trc(:,1),rcd_trc_next_u);
   hold on, plot(rcd_trc(:,1),rcd_trc_u);
   
end

if t>T-h
    massion = 1;
else
    massion = 0;
end
end


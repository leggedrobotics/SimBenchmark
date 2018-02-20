% analytic solution 
% g = 9.8;
% m = 1;
% n = 25;
% M = 10;
% F = 150;
% mu1 = 0.4; 
% mu2 = 0.8;
% 
% f1 = double(mu1 * (M + n * m)  * g); 
% f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M)); 
% a1 = double((F - f1 - n * f2) / M);
% a2 = double(f2 / m);
% 
% dt = 0.01;
% t = (1:3/dt) * dt;

% velocity ball
v_ball_mujoco = dlmread('./mujoco/0.010000_vel_ball.rlog','',4,0);

% position ball 
z_ball_mujoco = dlmread('./mujoco/0.010000_pos_ball.rlog','',4,0);

% 
% error_v_box_bullet = v_box_bullet(:,2) - v_box_true';
% error_v_box_rai = v_box_rai(:,2) - v_box_true';
% error_v_box_ode = v_box_ode(:,2) - v_box_true';

figure('Name', 'ball velocity')
plot(v_ball_mujoco)
% legend('bullet', 'rai', 'ode')

figure('Name', 'ball position')
plot(z_ball_mujoco)
% legend('true', 'raiSim', 'bulletSim')
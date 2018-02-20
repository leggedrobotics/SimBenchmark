%% path
data_path = '../data/rolling';

%% constants 
simTime = 3;

%% analytic solution 
g = 9.8;
m = 1;
n = 25;
M = 10;
F = 150;
mu1 = 0.4; 
mu2 = 0.8;

f1 = double(mu1 * (M + n * m)  * g); 
f2 = double(1 / M * (150 - f1) / (3.5 / m + 25 / M)); 
a1 = double((F - f1 - n * f2) / M);
a2 = double(f2 / m);

%% velocity 
dt = 0.01;
t = (1:simTime/dt) * dt;

% velocity box
v_box_true = a1 * t;
v_box_bullet = load('./bullet/001_box_vel.rlog');
v_box_rai = load('./rai/001_box_vel.rlog');
v_box_ode = load('./ode/001_box_vel.rlog');

error_v_box_bullet = v_box_bullet(:,2) - v_box_true';
error_v_box_rai = v_box_rai(:,2) - v_box_true';
error_v_box_ode = v_box_ode(:,2) - v_box_true';

figure('Name','box velocity')
plot(error_v_box_bullet)
hold on 
plot(error_v_box_rai)
plot(error_v_box_ode)
hold off
legend('bullet', 'rai', 'ode')

% figure(Name,'ball velocity')
% plot(v_box_true)
% hold on 
% plot(v_box_rai(:, 2))
% plot(v_box_bullet(:, 2))
% hold off
% legend('true', 'raiSim', 'bulletSim')
% 
% figure(Name,'box position')
% plot(v_box_true)
% hold on 
% plot(v_box_rai(:, 2))
% plot(v_box_bullet(:, 2))
% hold off
% legend('true', 'raiSim', 'bulletSim')
% 
% figure(Name,'ball position')
% plot(v_box_true)
% hold on 
% plot(v_box_rai(:, 2))
% plot(v_box_bullet(:, 2))
% hold off
% legend('true', 'raiSim', 'bulletSim')
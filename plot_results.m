function plot_results(t, q, state_hist, acc_nav, is_still)
% plot_results - attitude, position, velocity, acc and ZUPT

pos = state_hist(:,1:3);
vel = state_hist(:,4:6);

figure('Name','姿态（四元数）');
subplot(4,1,1); plot(t, q(:,1)); ylabel('qw'); grid on;
subplot(4,1,2); plot(t, q(:,2)); ylabel('qx'); grid on;
subplot(4,1,3); plot(t, q(:,3)); ylabel('qy'); grid on;
subplot(4,1,4); plot(t, q(:,4)); ylabel('qz'); xlabel('t (s)'); grid on;

figure('Name','位置和速度');
subplot(3,1,1); plot(t, pos(:,1)); ylabel('x (m)'); grid on;
subplot(3,1,2); plot(t, pos(:,2)); ylabel('y (m)'); grid on;
subplot(3,1,3); plot(t, pos(:,3)); ylabel('z (m)'); xlabel('t (s)'); grid on;

figure('Name','速度');
plot(t, vel); legend('vx','vy','vz'); xlabel('t (s)'); grid on;

figure('Name','加速度地面系 (扣重力)');
plot(t, acc_nav); legend('ax','ay','az'); xlabel('t (s)'); grid on;

figure('Name','静止检测 (ZUPT)');
plot(t, double(is_still)); ylim([-0.1 1.1]); xlabel('t (s)'); title('1: 静止'); grid on;
end


function q = estimate_attitude_enhanced(ax_c, ay_c, az_c, gx_c, gy_c, gz_c, t, beta)
% estimate_attitude_enhanced - Enhanced attitude estimation with adaptive parameters

N = length(t);
q = zeros(N, 4);

% Enhanced initial attitude estimation
useN = min(N, max(20, round(2/median(diff(t)))));  % Use 2 seconds or 20 samples
a0 = [mean(ax_c(1:useN)), mean(ay_c(1:useN)), mean(az_c(1:useN))];

% Robust initial attitude calculation
roll0  = atan2(a0(2), a0(3));
pitch0 = atan2(-a0(1), sqrt(a0(2)^2 + a0(3)^2));
yaw0 = 0;  % Assume initial yaw is zero

% Convert to quaternion
cy = cos(yaw0*0.5); sy = sin(yaw0*0.5);
cp = cos(pitch0*0.5); sp = sin(pitch0*0.5);
cr = cos(roll0*0.5); sr = sin(roll0*0.5);

q0w = cr*cp*cy + sr*sp*sy;
q0x = sr*cp*cy - cr*sp*sy;
q0y = cr*sp*cy + sr*cp*sy;
q0z = cr*cp*sy - sr*sp*cy;

q(1,:) = [q0w q0x q0y q0z];
q(1,:) = q(1,:) / norm(q(1,:));

fprintf('初始姿态: roll=%.1f°, pitch=%.1f°, yaw=%.1f°\n', ...
    roll0*180/pi, pitch0*180/pi, yaw0*180/pi);

% Adaptive beta parameter
beta_base = beta;
gyro_norm_threshold = 0.1;  % rad/s

% Main attitude estimation loop
for k = 2:N
    dtk = t(k) - t(k-1);
    
    % Adaptive beta based on gyroscope magnitude
    gyro_norm = norm([gx_c(k), gy_c(k), gz_c(k)]);
    if gyro_norm > gyro_norm_threshold
        % During motion, trust gyroscope more (lower beta)
        beta_adaptive = beta_base * 0.5;
    else
        % During quasi-static periods, trust accelerometer more (higher beta)
        beta_adaptive = beta_base * 1.2;
    end
    
    % Enhanced Madgwick AHRS with adaptive parameters
    q(k,:) = madgwick_ahrs_enhanced([ax_c(k) ay_c(k) az_c(k)], ...
                                   [gx_c(k) gy_c(k) gz_c(k)], ...
                                   q(k-1,:), beta_adaptive, dtk);
    
    % Periodic normalization to prevent drift
    if mod(k, 100) == 0
        q(k,:) = q(k,:) / norm(q(k,:));
    end
end

% Final normalization of all quaternions
for k = 1:N
    q(k,:) = q(k,:) / norm(q(k,:));
end

% Validate quaternion sequence for large jumps
for k = 2:N
    % Check for quaternion sign flip
    if dot(q(k,:), q(k-1,:)) < 0
        q(k,:) = -q(k,:);
    end
end

fprintf('姿态估计完成，使用自适应Madgwick算法\n');

end

function q_out = madgwick_ahrs_enhanced(acc, gyro, q_prev, beta, dt)
% Enhanced Madgwick AHRS with improved filtering and error handling

persistent gyro_filtered acc_filtered;

% Initialize filters
if isempty(gyro_filtered)
    gyro_filtered = gyro;
    acc_filtered = acc;
else
    % Adaptive filtering based on motion
    gyro_norm = norm(gyro);
    if gyro_norm > 0.2  % High motion
        alpha_gyro = 0.9;   % Less filtering during motion
        alpha_acc = 0.7;
    else  % Low motion
        alpha_gyro = 0.7;   % More filtering during quasi-static
        alpha_acc = 0.8;
    end
    
    gyro_filtered = alpha_gyro * gyro_filtered + (1-alpha_gyro) * gyro;
    acc_filtered = alpha_acc * acc_filtered + (1-alpha_acc) * acc;
end

% Use filtered data
ax = acc_filtered(1); ay = acc_filtered(2); az = acc_filtered(3);
gx = gyro_filtered(1); gy = gyro_filtered(2); gz = gyro_filtered(3);

% Normalize accelerometer measurement
normA = sqrt(ax*ax + ay*ay + az*az);
if normA < 1e-6  % Avoid division by zero
    q_out = q_prev; 
    return;
end

% Check for reasonable accelerometer measurement (should be close to 1g)
if normA < 5 || normA > 15  % Outside reasonable range for gravity measurement
    beta = beta * 0.1;  % Reduce trust in accelerometer
end

ax = ax / normA; 
ay = ay / normA; 
az = az / normA;

% Current quaternion components
q1 = q_prev(1); q2 = q_prev(2); q3 = q_prev(3); q4 = q_prev(4);

% Gradient descent algorithm correcting the 'direction' of the estimated gravity
f1 = 2*(q2*q4 - q1*q3) - ax;
f2 = 2*(q1*q2 + q3*q4) - ay;
f3 = 2*(0.5 - q2*q2 - q3*q3) - az;

% Jacobian matrix
J = [-2*q3,  2*q4, -2*q1,  2*q2;
      2*q2,  2*q1,  2*q4,  2*q3;
         0, -4*q2, -4*q3,     0];

% Gradient (objective function gradient)
grad = J' * [f1; f2; f3];
grad_norm = norm(grad);
if grad_norm > 1e-12
    grad = grad / grad_norm;
else
    grad = zeros(4,1);
end

% Rate of change of quaternion from gyroscope
qDot_omega = 0.5 * quatmultiply(q_prev, [0 gx gy gz]);

% Apply feedback step
qDot = qDot_omega - beta * grad';

% Integrate to yield quaternion
q = q_prev + qDot * dt;

% Normalize quaternion
q_norm = norm(q);
if q_norm > 1e-12
    q_out = q / q_norm;
else
    q_out = q_prev;  % Fallback to previous quaternion
end

end

function q = quatmultiply(q1, q2)
% Quaternion multiplication: q1 * q2
% q1 and q2 as 1x4 [w x y z]
w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);

w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
z = w1*z2 + x1*y2 - y1*x2 + z1*w2;

q = [w x y z];
end
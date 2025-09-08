function q_out = madgwick_ahrs(acc, gyro, q_prev, beta, dt)
% Madgwick AHRS (no magnetometer)
% q_prev: [w x y z]

persistent gyro_filtered;
if isempty(gyro_filtered)
    gyro_filtered = gyro(:)';
else
    alpha = 0.8; % low-pass
    gyro_filtered = alpha * gyro_filtered + (1-alpha) * gyro(:)';
end

gx = gyro_filtered(1); gy = gyro_filtered(2); gz = gyro_filtered(3);
ax = acc(1); ay = acc(2); az = acc(3);

normA = sqrt(ax*ax + ay*ay + az*az);
if normA < 1e-6
    q_out = q_prev; return; %#ok<RETURN>
end
ax = ax / normA; ay = ay / normA; az = az / normA;

q1 = q_prev(1); q2 = q_prev(2); q3 = q_prev(3); q4 = q_prev(4);

f1 = 2*(q2*q4 - q1*q3) - ax;
f2 = 2*(q1*q2 + q3*q4) - ay;
f3 = 2*(0.5 - q2*q2 - q3*q3) - az;
J = [ -2*q3,  2*q4, -2*q1, 2*q2;
        2*q2,  2*q1,  2*q4, 2*q3;
            0, -4*q2, -4*q3, 0   ];
grad = (J' * [f1; f2; f3]);
grad = grad / max(norm(grad), 1e-12);

qDot_omega = 0.5 * quatmultiply_local(q_prev, [0 gx gy gz]);
qDot = qDot_omega - beta * grad';
q = q_prev + qDot * dt;
q = q / max(norm(q), 1e-12);

q_out = q;
end

function q = quatmultiply_local(q1, q2)
w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
q = [w x y z];
end


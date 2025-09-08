function C = quat2dcm(q)
% quat2dcm - body->nav rotation matrix from quaternion [w x y z]
w = q(1); x = q(2); y = q(3); z = q(4);
C = [1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
     2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
     2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)];
end


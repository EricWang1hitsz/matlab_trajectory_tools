% orientation boxminus for quaternions 
function r = k_quat_diff(p,q)
% Quaternions are w x y z, active.

if(p(1)*q(1) < 0)
    p = -p;
end

r = wrapToPi(k_quat_log(k_quat_mult(p,k_quat_inv(q))));

end
% orientation boxminus for quaternions 
function r = k_quat_diff(p,q)
% Quaternions are w x y z, active.

r = abs(wrapToPi(norm(k_quat_diff(p,q))));

end
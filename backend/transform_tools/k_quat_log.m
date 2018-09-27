function r = k_quat_log(q)
% Quaternions are w x y z, active.

n=norm(q(2:4));
if n~=0
    r=2*atan2(n,q(1))*q(2:4)/n;
else
    r=[0 0 0];
end

end

function xi = se2_log(T)
%SE2_LOG  Log map from SE(2) to twist coordinates xi = [vx; vy; omega].
%
%   T = [ R  p;
%         0  1 ];
%
%   If theta ~= 0:
%       theta = atan2(R(2,1), R(1,1));
%       V = [ sin(theta)/theta     -(1-cos(theta))/theta;
%             (1-cos(theta))/theta  sin(theta)/theta ];
%       v = V^{-1} * p;
%   If theta ~ 0:
%       omega = 0; v = p.

    R = T(1:2,1:2);
    p = T(1:2,3);

    theta = atan2(R(2,1), R(1,1));
    eps_th = 1e-9;

    if abs(theta) < eps_th
        % Pure translation
        v = p;
        omega = 0;
    else
        A = sin(theta)/theta;
        B = (1 - cos(theta))/theta;
        V = [ A  -B;
              B   A ];
        v = V \ p;    % solve V*v = p
        omega = theta;
    end

    xi = [v; omega];
end

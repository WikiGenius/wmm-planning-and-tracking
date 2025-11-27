function T = se2_exp(xi, sigma)
%SE2_EXP  Exponential map on SE(2).
%
%   xi    = [vx; vy; omega]  (body twist at sigma=1)
%   sigma = scalar path parameter in [0,1]
%
%   T(sigma) = exp( (sigma * xi)^ )  in SE(2)

    vx = xi(1);
    vy = xi(2);
    w  = xi(3);

    eps_w = 1e-9;

    if abs(w) < eps_w
        % Pure translation
        p = sigma * [vx; vy];
        R = eye(2);
    else
        th = w * sigma;

        c = cos(th);
        s = sin(th);

        R = [ c  -s;
              s   c ];

        A = sin(th)/w;
        B = (1 - cos(th))/w;

        V = [ A  -B;
              B   A ];

        p = V * [vx; vy];
    end

    T = [ R  p;
          0  0  1 ];
end

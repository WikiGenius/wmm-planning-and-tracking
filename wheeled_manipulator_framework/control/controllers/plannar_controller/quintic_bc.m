function [q, qdot, qddot] = quintic_bc(t, Tf, q0, qf, qd0, qdf, qdd0, qddf)
%QUINTIC_BC
%  Quintic polynomial interpolation with:
%    q(0)=q0,   q(Tf)=qf
%    q'(0)=qd0, q'(Tf)=qdf
%    q''(0)=qdd0, q''(Tf)=qddf

    if Tf <= 0
        q     = q0;
        qdot  = qd0;
        qddot = qdd0;
        return;
    end

    T = Tf;
    T2 = T^2; T3 = T^3; T4 = T^4; T5 = T^5;

    A = [ ...
        1   0    0     0      0      0;     % q(0)
        0   1    0     0      0      0;     % q'(0)
        0   0    2     0      0      0;     % q''(0)
        1   T   T2    T3     T4     T5;     % q(Tf)
        0   1   2*T   3*T2   4*T3   5*T4;   % q'(Tf)
        0   0    2    6*T   12*T2  20*T3];  % q''(Tf)

    b = [q0; qd0; qdd0; qf; qdf; qddf];

    a = A \ b;  % coefficients

    t2 = t^2; t3 = t^3; t4 = t^4; t5 = t^5;

    q     = a(1) + a(2)*t + a(3)*t2 + a(4)*t3 + a(5)*t4 + a(6)*t5;
    qdot  =        a(2)   + 2*a(3)*t + 3*a(4)*t2 + 4*a(5)*t3 + 5*a(6)*t4;
    qddot =                 2*a(3)   + 6*a(4)*t + 12*a(5)*t2 + 20*a(6)*t3;
end

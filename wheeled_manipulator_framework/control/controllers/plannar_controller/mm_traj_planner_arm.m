function arm_ref = mm_traj_planner_arm(t, traj)
%MM_TRAJ_PLANNER_ARM
%  Quintic time-scaling planner for the 3-DOF arm only.
%
%  Inputs:
%    t    : current time
%    traj : scenario (q0_arm, qf_arm, qd0_arm, qdf_arm, etc., Tf)
%
%  Output:
%    arm_ref struct with fields:
%      q     : 3x1 joint positions
%      qdot  : 3x1 joint velocities
%      qddot : 3x1 joint accelerations

    Tf   = traj.Tf;
    t_cl = max(0, min(t, Tf));   % clamp time

    % Persistent coefficients: compute once, reuse
    persistent coefs_cached Tf_cached

    if isempty(coefs_cached) || isempty(Tf_cached) || Tf_cached ~= Tf
        % Recompute quintic coefficients for each joint
        q0   = traj.q0_arm(:);    % 3x1
        qf   = traj.qf_arm(:);    % 3x1
        qd0  = traj.qd0_arm(:);   % 3x1
        qdf  = traj.qdf_arm(:);   % 3x1
        qdd0 = traj.qdd0_arm(:);  % 3x1
        qddf = traj.qddf_arm(:);  % 3x1

        coefs_cached = zeros(3, 6);  % each row: [a0 a1 a2 a3 a4 a5]

        T = Tf;
        T2 = T^2; T3 = T^3; T4 = T^4; T5 = T^5;

        for i = 1:3
            a0 = q0(i);
            a1 = qd0(i);
            a2 = qdd0(i)/2;

            % Solve for a3, a4, a5 using final conditions
            M = [ T3   T4    T5;
                  3*T2 4*T3  5*T4;
                  6*T  12*T2 20*T3 ];

            b = [ qf(i)  - (a0 + a1*T + a2*T2);
                  qdf(i) - (a1 + 2*a2*T);
                  qddf(i)- (2*a2) ];

            a345 = M \ b;

            coefs_cached(i,:) = [a0 a1 a2 a345(:).'];
        end

        Tf_cached = Tf;
    end

    % Evaluate quintic at t_cl
    q     = zeros(3,1);
    qdot  = zeros(3,1);
    qddot = zeros(3,1);

    for i = 1:3
        a = coefs_cached(i,:);  % [a0 a1 a2 a3 a4 a5]
        a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4); a4 = a(5); a5 = a(6);

        tt  = t_cl;
        tt2 = tt^2; tt3 = tt^3; tt4 = tt^4; tt5 = tt^5;

        qi     = a0 + a1*tt + a2*tt2 + a3*tt3 + a4*tt4 + a5*tt5;
        qdi    =       a1   + 2*a2*tt + 3*a3*tt2 + 4*a4*tt3 + 5*a5*tt4;
        qddi   =            2*a2      + 6*a3*tt  + 12*a4*tt2 + 20*a5*tt3;

        q(i)     = qi;
        qdot(i)  = qdi;
        qddot(i) = qddi;
    end

    arm_ref = struct( ...
        'q',     q, ...
        'qdot',  qdot, ...
        'qddot', qddot);
end

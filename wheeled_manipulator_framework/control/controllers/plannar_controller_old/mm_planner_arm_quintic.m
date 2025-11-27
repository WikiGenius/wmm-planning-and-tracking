function arm_ref = mm_planner_arm_quintic(t, Tf, traj)
%MM_PLANNER_ARM_QUINTIC
%  Per-joint quintic interpolation for a 3-DOF arm:
%    q(0)=q0, q(Tf)=qf,
%    qdot(0)=qd0, qdot(Tf)=qdf,
%    qddot(0)=qdd0, qddot(Tf)=qddf.

    q0   = traj.q0_arm(:);
    qf   = traj.qf_arm(:);
    qd0  = traj.qd0_arm(:);
    qdf  = traj.qdf_arm(:);
    qdd0 = traj.qdd0_arm(:);
    qddf = traj.qddf_arm(:);

    nq = numel(q0);
    q     = zeros(nq,1);
    qdot  = zeros(nq,1);
    qddot = zeros(nq,1);

    for i = 1:nq
        [q(i), qdot(i), qddot(i)] = quintic_bc( ...
            t, Tf, ...
            q0(i),  qf(i), ...
            qd0(i), qdf(i), ...
            qdd0(i), qddf(i));
    end

    arm_ref = struct('q', q, 'qdot', qdot, 'qddot', qddot);
end

function qdot_cmd = mm_kin_ctrl_arm_pd(q_arm, qdot_arm, arm_ref, Kp_arm, Kd_arm)
%MM_KIN_CTRL_ARM_PD
%  Velocity-level PD kinematic controller for a multi-DOF arm.
%
%  Inputs:
%    q_arm     : current joint positions (n x 1)
%    qdot_arm  : current joint velocities (n x 1)
%    arm_ref   : struct with fields:
%                  q    (desired joint positions, n x 1)
%                  qdot (desired joint velocities, n x 1)
%    Kp_arm    : proportional gain matrix (n x n), scalar, or n-element vector
%    Kd_arm    : derivative gain matrix (n x n), scalar, or n-element vector
%
%  Output:
%    qdot_cmd  : commanded joint velocities (n x 1)

    % Ensure column vectors
    q_arm    = q_arm(:);
    qdot_arm = qdot_arm(:);
    q_ref    = arm_ref.q(:);
    qd_ref   = arm_ref.qdot(:);

    n = numel(q_arm);
    if numel(q_ref) ~= n || numel(qd_ref) ~= n
        error('mm_kin_ctrl_arm_pd: dimension mismatch between arm_ref and q_arm.');
    end

    % Expand / interpret Kp_arm
    if isscalar(Kp_arm)
        Kp = Kp_arm * eye(n);
    elseif isvector(Kp_arm) && numel(Kp_arm) == n
        Kp = diag(Kp_arm(:));
    else
        Kp = Kp_arm;
    end

    % Expand / interpret Kd_arm
    if isscalar(Kd_arm)
        Kd = Kd_arm * eye(n);
    elseif isvector(Kd_arm) && numel(Kd_arm) == n
        Kd = diag(Kd_arm(:));
    else
        Kd = Kd_arm;
    end

    % Check shapes
    if ~all(size(Kp) == [n n]) || ~all(size(Kd) == [n n])
        error('mm_kin_ctrl_arm_pd: Kp_arm and Kd_arm must be n x n, scalar, or n-vector.');
    end

    % Errors
    e_q    = q_ref  - q_arm;
    e_qdot = qd_ref - qdot_arm;

    % Velocity-level PD with feedforward
    qdot_cmd = qd_ref + Kp*e_q + Kd*e_qdot;
end

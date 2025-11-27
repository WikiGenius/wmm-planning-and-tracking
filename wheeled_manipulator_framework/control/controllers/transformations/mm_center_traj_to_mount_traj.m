function mount = mm_center_traj_to_mount_traj(center, offset)
%MM_CENTER_TRAJ_TO_MOUNT_TRAJ
% Convert full trajectory from CENTER (M) → MOUNT point (B).
%
% Expected fields in `center`:
%   center.x,  center.y,  center.theta
%   center.x_dot,  center.y_dot,  center.theta_dot
%   center.x_ddot, center.y_ddot, center.theta_ddot
%
% Offset:
%   offset.dx, offset.dy : vector from CENTER M to MOUNT B (body frame)

    % Pose
    [xb, yb, thetab] = mm_center_to_mount( ...
        center.x, center.y, center.theta, offset);

    % Velocity
    [xb_dot, yb_dot, thetab_dot] = mm_center_vel_to_mount_vel( ...
        center.x_dot, center.y_dot, center.theta, center.theta_dot, offset);

    % Acceleration
    [xb_ddot, yb_ddot, thetab_ddot] = mm_center_acc_to_mount_acc( ...
        center.x_ddot, center.y_ddot, center.theta, ...
        center.theta_dot, center.theta_ddot, offset);

    % Pack result
    mount.x      = xb;
    mount.y      = yb;
    mount.theta  = thetab;

    mount.x_dot      = xb_dot;
    mount.y_dot      = yb_dot;
    mount.theta_dot  = thetab_dot;

    mount.x_ddot      = xb_ddot;
    mount.y_ddot      = yb_ddot;
    mount.theta_ddot  = thetab_ddot;
end

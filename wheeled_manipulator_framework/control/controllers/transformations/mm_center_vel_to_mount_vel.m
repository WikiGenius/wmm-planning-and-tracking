function [xB_dot, yB_dot, thetaB_dot] = ...
    mm_center_vel_to_mount_vel(xM_dot, yM_dot, thetaM, thetaM_dot, offset)
%MM_CENTER_VEL_TO_MOUNT_VEL
% Convert base-center velocity → mount-point velocity in the world frame.
%
% Geometry:
%   B is the mounting point, M is the geometric center.
%   offset = [dx; dy] is the vector from CENTER M to MOUNT B
%   expressed in the body frame (M–frame).
%
% Inputs:
%   xM_dot, yM_dot : time-derivative of center position in world frame
%   thetaM         : base heading (center frame) in world frame
%   thetaM_dot     : yaw rate
%   offset.dx      : x-offset from M to B in body frame
%   offset.dy      : y-offset from M to B in body frame
%
% Outputs:
%   xB_dot, yB_dot : velocity of mount point B in world frame
%   thetaB_dot     : yaw rate at B (same as at M)

    % Unpack offset
    dx = offset.dx;
    dy = offset.dy;

    % Precompute rotation terms
    c = cos(thetaM);
    s = sin(thetaM);

    % Velocity mapping:
    % v_B = v_M + ω × r_MB  (in 2D, r_MB = [dx; dy] in body frame)
    xB_dot = xM_dot - thetaM_dot*(dx*s + dy*c);
    yB_dot = yM_dot + thetaM_dot*(dx*c - dy*s);

    % Yaw rate is the same everywhere on a rigid body
    thetaB_dot = thetaM_dot;
end

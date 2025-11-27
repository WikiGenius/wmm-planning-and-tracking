function [xm_dot, ym_dot, thetam_dot] = ...
    mm_mount_vel_to_center_vel(xb_dot, yb_dot, thetab, thetab_dot, offset)
%MM_MOUNT_VEL_TO_CENTER_VEL
% Convert mount velocity → center velocity.

dx = offset.dx;
dy = offset.dy;

c = cos(thetab);
s = sin(thetab);

xm_dot = xb_dot + thetab_dot*(dx*s + dy*c);
ym_dot = yb_dot - thetab_dot*(dx*c - dy*s);
thetam_dot = thetab_dot;

end

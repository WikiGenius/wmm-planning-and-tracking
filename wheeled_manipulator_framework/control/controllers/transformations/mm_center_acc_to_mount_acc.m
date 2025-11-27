function [xb_ddot, yb_ddot, thetab_ddot] = ...
    mm_center_acc_to_mount_acc(xm_ddot, ym_ddot, thetam, thetam_dot, thetam_ddot, offset)
%MM_CENTER_ACC_TO_MOUNT_ACC
% Convert center acceleration → mount point acceleration.

dx = offset.dx;
dy = offset.dy;

c = cos(thetam);
s = sin(thetam);

xb_ddot = xm_ddot ...
          - thetam_ddot*(dx*s + dy*c) ...
          - (thetam_dot^2)*(dx*c - dy*s);

yb_ddot = ym_ddot ...
          + thetam_ddot*(dx*c - dy*s) ...
          - (thetam_dot^2)*(dx*s + dy*c);

thetab_ddot = thetam_ddot;
end

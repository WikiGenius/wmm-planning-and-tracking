function [xm_ddot, ym_ddot, thetam_ddot] = ...
    mm_mount_acc_to_center_acc(xb_ddot, yb_ddot, thetab, thetab_dot, thetab_ddot, offset)
%MM_MOUNT_ACC_TO_CENTER_ACC
% Convert mount point acceleration → center acceleration.

dx = offset.dx;
dy = offset.dy;

c = cos(thetab);
s = sin(thetab);

xm_ddot = xb_ddot ...
          + thetab_ddot*(dx*s + dy*c) ...
          + (thetab_dot^2)*(dx*c - dy*s);

ym_ddot = yb_ddot ...
          - thetab_ddot*(dx*c - dy*s) ...
          + (thetab_dot^2)*(dx*s + dy*c);

thetam_ddot = thetab_ddot;

end

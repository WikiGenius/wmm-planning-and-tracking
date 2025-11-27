function [xB, yB, thetaB] = mm_center_to_mount(xM, yM, thetaM, offset)
%MM_CENTER_TO_MOUNT
%  Convert base pose from center frame M → mounting frame B.
%
%  Inputs:
%    xM, yM, thetaM : pose at the center M
%    offset.dx, offset.dy : vector from CENTER M to MOUNT B,
%                           expressed in body frame.
%
%  Output:
%    xB, yB, thetaB : pose of the mount B in world frame

    dx = offset.dx;
    dy = offset.dy;

    c = cos(thetaM);
    s = sin(thetaM);

    xB = xM + dx*c - dy*s;
    yB = yM + dx*s + dy*c;
    thetaB = thetaM;
end

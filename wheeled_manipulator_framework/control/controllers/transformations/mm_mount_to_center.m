function [xM, yM, thetaM] = mm_mount_to_center(xB, yB, thetaB, offset)
%MM_MOUNT_TO_CENTER
%  Convert base pose from mounting frame B → center frame M.
%
%  Inputs:
%    xB, yB, thetaB : pose at the mounting point B
%    offset.dx, offset.dy : vector from CENTER M to MOUNT B,
%                           expressed in body frame.
%
%  Output:
%    xM, yM, thetaM : pose of the center M in world frame

    dx = offset.dx;
    dy = offset.dy;

    c = cos(thetaB);
    s = sin(thetaB);

    % B is at:
    %   xB = xM + dx*cos(theta) - dy*sin(theta)
    %   yB = yM + dx*sin(theta) + dy*cos(theta)
    %
    % Solve for (xM, yM):
    xM = xB - dx*c + dy*s;
    yM = yB - dx*s - dy*c;
    thetaM = thetaB;
end

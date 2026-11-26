function [R,M,h,G] = robot_model(q,u,p)
% q = [axle x,y,yaw,right wheel,left wheel,arm yaw,shoulder,elbow].
% u = qdot(4:8). Each column of J maps u to a world COM velocity.
[R,J] = geometry(q,p);
velocity = R*u;
% Complex-step directional differentiation avoids subtractive cancellation.
[~,Jpert] = geometry(q + 1i*1e-20*velocity,p);
Jdot = imag(Jpert)/1e-20;
mass = repelem([p.m_w,p.m_w,p.m_m,p.m1,p.m2,p.m3],3).';
yaw = R(3,:);
spin = eye(5);
armYaw = yaw + spin(3,:);
pitch = spin(4,:);
elbow = pitch + spin(5,:);
% Constant effective rotational inertias of the course model.
M = J.'*(mass.*J) + (p.I_m+2*p.I_yyw)*(yaw.'*yaw) ...
    + p.I_zzw*diag([1 1 0 0 0]) ...
    + (p.I_zz1+p.I_zz2+p.I_zz3)*(armYaw.'*armYaw) ...
    + p.I_yy2*(pitch.'*pitch) + p.I_yy3*(elbow.'*elbow);
h = J.'*(mass.*(Jdot*u));
G = J(3:3:end,:).'*(mass(3:3:end)*p.g);
end

function [R,J] = geometry(q,p)
f = [cos(q(3));sin(q(3));0];
left = [-f(2);f(1);0];
forward = p.r/2*[1 1 0 0 0];
yaw = p.r/(2*p.L)*[1 -1 0 0 0];
axle = f*forward;
R = [axle(1:2,:);yaw;eye(5)];
mount = axle + p.d*left*yaw;
az = q(3)+q(6);
dir = [cos(az);sin(az);0];
side = [-sin(az);cos(az);0];
up = [0;0;1];
armYaw = yaw + [0 0 1 0 0];
pitch = [0 0 0 1 0];
combined = [0 0 0 1 1];
b = q(7); c = b+q(8);
segment = @(len,ang,rate) len*(cos(ang)*side*armYaw ...
    + (-sin(ang)*dir+cos(ang)*up)*rate);
shoulder = mount + p.L_1*side*armYaw;
J = [axle+p.L*f*yaw; axle-p.L*f*yaw; axle+p.L_cmx_m*left*yaw;
     mount+p.L_cmx_1*side*armYaw;
     shoulder+segment(p.L_cmx_2,b,pitch);
     shoulder+segment(p.L_2,b,pitch)+segment(p.L_cmx_3,c,combined)];
end

function verify_model(p)
% Independent physical identities and reference consistency checks.
for k=1:32
    q=sin((1:8)'*k/7); u=cos((1:5)'*k/3);
    [R,M,c]=robot_model(q,u,p);
    h=q(3); A=[-sin(h),cos(h),0,0,0;cos(h),sin(h),0,-p.r/2,-p.r/2;0,0,1,-p.r/(2*p.L),p.r/(2*p.L)];
    assert(norm(A*R(1:5,:),'fro')<1e-12);
    assert(norm(M-M.','fro')<1e-12 && min(eig(M))>0);
    step=1e-6; dq=R*u;
    [~,plus]=robot_model(q+step*dq,u,p);
    [~,minus]=robot_model(q-step*dq,u,p);
    assert(abs(u.'*((plus-minus)/(4*step)*u-c))<1e-8);
    % Independently differentiate full-coordinate kinetic energy and project
    % the Euler-Lagrange force for a nonzero reduced acceleration.
    udot=sin((1:5)'*k);
    [Rp,~]=robot_model(q+step*dq,u,p);
    [Rm,~]=robot_model(q-step*dq,u,p);
    ddq=R*udot+(Rp-Rm)*u/(2*step);
    F=full_mass(q,p);
    Fdot=(full_mass(q+step*dq,p)-full_mass(q-step*dq,p))/(2*step);
    grad=zeros(8,1);
    for axis=1:8
        offset=zeros(8,1); offset(axis)=step;
        grad(axis)=dq.'*(full_mass(q+offset,p)-full_mass(q-offset,p))*dq/(4*step);
    end
    assert(norm(R.'*(F*ddq+Fdot*dq-grad)-(M*udot+c))<1e-7);
    t=k/3.3;
    [~,v,a]=reference(t,p);
    [hi,vhi]=reference(t+step,p); [lo,vlo]=reference(t-step,p);
    assert(norm((hi-lo)/(2*step)-v)<1e-6 && norm((vhi-vlo)/(2*step)-a)<1e-6);
end
[R,~,~,G]=robot_model(zeros(8,1),zeros(5,1),p);
spin=R*[1;-1;0;0;0]; assert(norm(spin(1:2))<1e-12);
assert(norm(G(4:5)-p.g*[p.m2*p.L_cmx_2+p.m3*(p.L_2+p.L_cmx_3); p.m3*p.L_cmx_3])<1e-10);
end

function F=full_mass(q,p)
J=zeros(18,8);
for j=1:8
    probe=q; probe(j)=probe(j)+1i*1e-20;
    J(:,j)=imag(centers(probe,p))/1e-20;
end
m=repelem([p.m_w,p.m_w,p.m_m,p.m1,p.m2,p.m3],3).';
% Angular velocities in full coordinates for the effective-inertia model.
E=eye(8);
W=[E(3,:);E(4,:);E(5,:);E(3,:)+E(6,:);E(7,:);E(7,:)+E(8,:)];
inertias=[p.I_m+2*p.I_yyw;p.I_zzw;p.I_zzw;p.I_zz1+p.I_zz2+p.I_zz3;p.I_yy2;p.I_yy3];
F=J.'*(m.*J)+W.'*(inertias.*W);
end

function v=centers(q,p)
origin=[q(1:2);p.r]; f=[cos(q(3));sin(q(3));0];
left=[-f(2);f(1);0]; yaw=q(3)+q(6);
link=@(length,b) length*[cos(yaw)*cos(b);sin(yaw)*cos(b);sin(b)];
mount=origin+p.d*f+[0;0;p.L_0]; shoulder=mount+link(p.L_1,0);
v=[origin-p.L*left;origin+p.L*left;origin+p.L_cmx_m*f;
   mount+link(p.L_cmx_1,0); shoulder+link(p.L_cmx_2,q(7));
   shoulder+link(p.L_2,q(7))+link(p.L_cmx_3,q(7)+q(8))];
end

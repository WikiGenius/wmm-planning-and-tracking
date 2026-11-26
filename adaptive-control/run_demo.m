function results = run_demo(showAnimation)
% Run all three deterministic comparisons. Pass true for 3-D playback.
if nargin==0, showAnimation=false; end
root=fileparts(mfilename('fullpath'));
cfg=jsondecode(fileread(fullfile(root,'parameters.json')));
p=cfg.phys;
verify_model(p);
names=["moderate","high","sinusoidal"];
t=(0:0.01:10).';
opts=odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.02);
data=zeros(3,7);
for k=1:3
    for adaptive=0:1
        x0=[-p.d;0;0;0;0;0;-0.3;0.5;zeros(10,1)];
        [~,x]=ode45(@(t,x) dynamics(t,x,p,k,adaptive),t,x0,opts);
        error=zeros(size(t)); torque=error; power=error;
        for j=1:numel(t)
            [~,tau,e]=dynamics(t(j),x(j,:).',p,k,adaptive);
            error(j)=norm(e); torque(j)=norm(tau);
            power(j)=tau.'*x(j,9:13).';
        end
        assert(all(isfinite(x),'all'));
        col=1+adaptive;
        runs(col)=struct('state',x,'error',error,'torque',torque);
        data(k,col)=sqrt(trapz(t,error.^2)/10);
        data(k,col+3)=sqrt(trapz(t,torque.^2)/10);
        data(k,col+5)=trapz(t,power);
    end
    data(k,3)=100*(1-data(k,2)/data(k,1));
    if k==2, high=runs; end
end
results=array2table(data,'VariableNames',{'NominalRMS_rad','AdaptiveRMS_rad',...
    'Reduction_pct','NominalTorqueRMS_Nm','AdaptiveTorqueRMS_Nm',...
    'NominalWork_J','AdaptiveWork_J'},'RowNames',cellstr(names));
disp(results);
writetable(results,fullfile(root,'results.csv'),'WriteRowNames',true);
figure('Color','w','Position',[100 100 960 390]);
tiledlayout(1,2);
nexttile; hold on
for run=high
    plot(run.state(:,1),run.state(:,2),'LineWidth',2);
end
plot([-p.d 2],[0 2-p.d],'k--');
axis equal; grid on; xlabel('Axle x (m)'); ylabel('Axle y (m)');
title('Axle path'); legend('Nominal','Adaptive','Reference','Location','best');
nexttile; plot(t,[high(1).error high(2).error],'LineWidth',2);
grid on; xlabel('Time (s)'); ylabel('Joint-space error norm (rad)');
title('Tracking under asymmetric torque bias'); legend('Nominal','Adaptive');
exportgraphics(gcf,fullfile(root,'comparison.png'),'Resolution',140);
if showAnimation, playback(high(2).state,p,t); end
end

function [dx,tau,e]=dynamics(t,x,p,k,adaptive)
q=x(1:8); u=x(9:13); estimate=x(14:18);
[R,M,c,G]=robot_model(q,u,p);
[ref,rate,acc]=reference(t,p);
lambda=[2;2;1.5;1.5;1.5]; e=q(4:8)-ref; ed=u-rate;
s=ed+lambda.*e;
if k==1
    dist=[.15;.15;.02;.02;.02]; gain=[.08;.08;.03;.03;.03]; sigma=.03;
    limit=[2;2;.8;.8;.8];
elseif k==2
    dist=[.9;.1;.1;.1;.1]; gain=[.12;.12;.05;.05;.05]; sigma=.03;
    limit=[4;4;1.5;1.5;1.5];
else
    z=min(1,max(0,t)); envelope=z^3*(10-15*z+6*z^2);
    dist=envelope*[.5;.5;.1;.1;.1].*sin(2*pi*[.2;.3;.45;.55;.7]*t+[0;pi/4;pi/3;pi/2;2*pi/3]);
    gain=[.14;.14;.06;.06;.06]; sigma=.04; limit=[5;5;2;2;2];
end
damping=p.damping(:).*u;
tau=M*(acc-lambda.*ed-lambda.*s)+c+damping+G-adaptive*estimate;
update=adaptive*(gain.*(M.'\s)-sigma*estimate);
update(abs(estimate)>=limit & estimate.*update>0)=0;
dx=[R*u; M\(tau+dist-c-damping-G); update];
end

function playback(states,p,t)
figure('Color','w');
for k=1:5:numel(t)
    q=states(k,1:8); a=q(3)+q(6);
    axle=[q(1:2),0]; mount=axle+[p.d*cos(q(3)),p.d*sin(q(3)),p.r+p.L_0];
    link=@(len,b) len*[cos(a)*cos(b),sin(a)*cos(b),sin(b)];
    shoulder=mount+link(p.L_1,0); elbow=shoulder+link(p.L_2,q(7));
    points=[axle;mount;shoulder;elbow;elbow+link(p.L_3,q(7)+q(8))];
    plot3(states(1:k,1),states(1:k,2),zeros(k,1),'b-'); hold on
    plot3(points(:,1),points(:,2),points(:,3),'o-','LineWidth',3); hold off
    axis equal; axis([-1 3 -1 3 -0.5 1.5]); grid on; view(35,25);
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    title(sprintf('Adaptive tracking   %.2f s',t(k))); drawnow;
end
end

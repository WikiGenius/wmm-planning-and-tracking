function results = run_demo(showAnimation,gifPath,outputDir)
% Run all comparisons. Pass true for 3-D playback; optionally give a GIF path.
if nargin<1, showAnimation=false; end
if nargin<2, gifPath=''; end
root=fileparts(mfilename('fullpath'));
if nargin<3 || isempty(outputDir), outputDir=root; end
if ~isfolder(outputDir), mkdir(outputDir); end
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
writetable(results,fullfile(outputDir,'results.csv'),'WriteRowNames',true);
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
exportgraphics(gcf,fullfile(outputDir,'comparison.png'),'Resolution',140);
if showAnimation || ~isempty(gifPath), playback_comparison(high,p,t,gifPath); end
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

function playback_comparison(high,p,t,gifPath)
% The two state histories come from the same high-disturbance simulation.
f=figure('Color','w','Position',[100 100 900 580]);
ax=axes(f); hold(ax,'on'); grid(ax,'on'); view(ax,35,25);
colors=[0.12 0.35 0.85; 0.85 0.16 0.16];
h=gobjects(3,1);
h(1)=plot3(ax,[-p.d 2],[0 2-p.d],[0 0],'k--','LineWidth',1.4);
for j=1:2
    q=high(j).state;
    h(j+1)=plot3(ax,q(:,1),q(:,2),zeros(size(t)),'Color',colors(j,:),'LineWidth',1.7);
end
body=gobjects(2,1); arm=body; marker=body;
for j=1:2
    body(j)=plot3(ax,nan,nan,nan,'Color',colors(j,:),'LineWidth',2,'HandleVisibility','off');
    arm(j)=plot3(ax,nan,nan,nan,'o-','Color',colors(j,:),'LineWidth',2.5,'HandleVisibility','off');
    marker(j)=plot3(ax,nan,nan,nan,'o','MarkerFaceColor',colors(j,:),'Color',colors(j,:),'HandleVisibility','off');
end
axis(ax,'equal'); axis(ax,[-0.9 3 -0.8 2.8 -0.1 1.25]);
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
legend(ax,h,{'Reference axle path','Nominal','Adaptive'},'Location','northeast');
title(ax,'Nominal vs adaptive tracking');
indices=unique([1:10:numel(t),numel(t)]);
for n=1:numel(indices)
    k=indices(n);
    if ~isgraphics(f), return; end
    for j=1:2
        q=high(j).state(k,1:8);
        yaw=q(3); a=yaw+q(6);
        R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
        footprint=[-.18 .39 .39 -.18 -.18;-.26 -.26 .26 .26 -.26];
        xy=q(1:2).'+R*footprint;
        set(body(j),'XData',xy(1,:),'YData',xy(2,:),'ZData',p.r*ones(1,5));
        axle=[q(1:2),0]; mount=axle+[p.d*cos(yaw),p.d*sin(yaw),p.r+p.L_0];
        link=@(len,b) len*[cos(a)*cos(b),sin(a)*cos(b),sin(b)];
        shoulder=mount+link(p.L_1,0); elbow=shoulder+link(p.L_2,q(7));
        points=[mount-[0 0 p.L_0];mount;shoulder;elbow;elbow+link(p.L_3,q(7)+q(8))];
        set(arm(j),'XData',points(:,1),'YData',points(:,2),'ZData',points(:,3));
        set(marker(j),'XData',q(1),'YData',q(2),'ZData',0);
    end
    title(ax,sprintf('High asymmetric torque bias | t = %.2f s',t(k)));
    drawnow;
    if ~isempty(gifPath)
        [im,map]=rgb2ind(frame2im(getframe(f)),256);
        if n==1
            imwrite(im,map,gifPath,'gif','LoopCount',Inf,'DelayTime',0.1);
        else
            imwrite(im,map,gifPath,'gif','WriteMode','append','DelayTime',0.1);
        end
    end
end
end

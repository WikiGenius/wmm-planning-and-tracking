function [pos,vel,acc] = reference(t,p)
% Rotate, translate, rotate between mount poses [0,0,0] and [2,2,pi/2].
delta = [2+p.d;2-p.d];
heading = atan2(delta(2),delta(1));
turns = [heading,0,pi/2-heading];
travel = [0,norm(delta),0];
duration = 10*(abs(turns)+travel)/sum(abs(turns)+travel);
starts = [0,cumsum(duration(1:2))];
profile = quintic((t-starts)./duration,duration);
wheelChange = [(travel+p.L*turns);(travel-p.L*turns)]/p.r;
wheel = wheelChange*profile;
arm = [2;0.5;-0.9]*quintic(t/10,10);
arm(:,1) = arm(:,1)+[0;-0.3;0.5];
all = [wheel;arm];
pos=all(:,1); vel=all(:,2); acc=all(:,3);
end

function profile = quintic(z,T)
z=min(1,max(0,z(:))); T=T(:);
profile=[z.^3.*(10-15*z+6*z.^2),30*z.^2.*(1-z).^2./T,...
    60*z.*(1-z).*(1-2*z)./T.^2];
end

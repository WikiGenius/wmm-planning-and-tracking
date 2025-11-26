function [s, sdot, sddot] = mm_quintic_timescaling(t, Tf)
%MM_QUINTIC_TIMESCALING
%  Quintic time-scaling s(t) in [0,1] with:
%    s(0)=0, s(Tf)=1
%    sdot(0)=sdot(Tf)=0
%    sddot(0)=sddot(Tf)=0

    if Tf <= 0
        s = 0; sdot = 0; sddot = 0;
        return;
    end

    tau = t / Tf;

    s     = 10*tau^3 - 15*tau^4 + 6*tau^5;
    sdot  = (30*tau^2 - 60*tau^3 + 30*tau^4) / Tf;
    sddot = (60*tau - 180*tau^2 + 120*tau^3) / (Tf^2);
end

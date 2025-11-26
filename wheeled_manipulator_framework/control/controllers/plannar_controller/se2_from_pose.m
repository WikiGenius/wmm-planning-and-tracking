function T = se2_from_pose(pose)
%SE2_FROM_POSE  Build SE(2) homogeneous transform from [x; y; theta].

    x  = pose(1);
    y  = pose(2);
    th = pose(3);

    c = cos(th);
    s = sin(th);

    T = [ c  -s   x;
          s   c   y;
          0   0   1 ];
end

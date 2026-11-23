function a = cl_wrap(a)
%CL_WRAP  Wrap an angle to [-pi, pi].
a = atan2(sin(a), cos(a));
end

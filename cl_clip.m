function y = cl_clip(x, lo, hi)
%CL_CLIP  Clamp x to [lo, hi].
y = min(max(x, lo), hi);
end

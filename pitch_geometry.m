function geom = pitch_geometry(V, D, mode)
%PITCH_GEOMETRY  Rod inertia data for one closed-loop configuration.
%
%   Stack length and the ascent static margin come from vehicle_params.
%   The ship uses the Starship length from that same library and the same
%   ascent margin, because the pitch study does not publish a separate
%   ship margin. arm and inertia stay empty so ascent_dynamics uses
%   m*L^2/12 and L/2 unless a step-4 hook fills them.

if strcmp(mode, 'ship')
    geom.L = V.library.starship.L;
    geom.config = 'ship';
else
    geom.L = V.stackLength;
    geom.config = 'stack';
end
geom.CNa = V.CNa;
geom.area = V.area;
geom.diam = V.diam;
geom.lcp = -V.staticMarginAscent * V.diam;
geom.arm = [];
geom.inertia = [];
geom.actWn = D.actWn;
geom.actZeta = D.actZeta;
geom.gimbalRate = D.gimbalRate;
geom.deltaMax = D.deltaMax;
end

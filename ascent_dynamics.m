function [dY, aux] = ascent_dynamics(t, Y, S, E, gravMu, mode, geom, deltaCmd, wDown)
%ASCENT_DYNAMICS  Point-mass ascent plus the pitch states the controller flies.
%
%   [dY, aux] = ascent_dynamics(t, Y, S, E, gravMu, mode, geom, deltaCmd, wDown)
%
%   Y = [x; h; vx; vh; m; theta; omega; delta; rate]
%     theta   nose angle from local vertical, positive downrange [rad]
%     omega   inertial pitch rate [rad/s]
%     delta   gimbal, controller sign: positive delta increases theta [rad]
%     rate    gimbal rate [rad/s]
%
%   Translational force and mass flow match starshipStack when the nose is
%   on the guidance attitude, the gimbal is zero, and the wind is zero.
%   Thrust direction is the vehicle attitude plus the gimbal. Angle of
%   attack adds a normal force (lift and extra drag) and the pitching
%   moment q*CNa*A*lcp*sin(alpha). The actuator is the same second-order
%   servo as boosterPitchTVC, with rate and angle stops.
%
%   wDown is horizontal wind, positive downrange, the same sign as
%   matlab-rocket-sim P.wind. The controls-repo gust is applied with the
%   opposite sign so a positive gustSpeed reduces alpha, matching
%   gustAccPerWind = -lambda/V.

x = Y(1); %#ok<NASGU>
h = Y(2);
vx = Y(3);
vh = Y(4);
m = max(Y(5), 1);
theta = Y(6);
omega = Y(7);
delta = Y(8);
rate = Y(9);

gcmd = guidance_command(t, Y, S, E, gravMu, mode);
r = gcmd.r;
vrx = vx - wDown;
vrh = vh;
vr = hypot(vrx, vrh);
if vr > 1
    gammaAir = atan2(vrx, vrh);
else
    gammaAir = 0;
end
alpha = cl_wrap(theta - gammaAir);

Cd = S.Cd;
if isfield(E, 'machDrag') && E.machDrag
    Cd = Cd * machDrag(vr / max(gcmd.aSound, 1));
end
q = 0.5 * gcmd.rho * vr * vr;
Dmag = q * Cd * S.A;
if vr > 0
    aDrag = -Dmag * [vrx; vrh] / (vr * m);
else
    aDrag = [0; 0];
end

aero = sin(alpha);
p = [cos(theta); -sin(theta)];
Fn = -q * geom.CNa * geom.area * aero;
aLift = Fn * p / m;

u = [sin(theta + delta); cos(theta + delta)];
aThrust = gcmd.F * u / m;
aNG = aThrust + aDrag + aLift;

[I, arm] = inertia_arm(geom, m);
M_tvc = gcmd.F * arm * sin(delta);
M_aero = q * geom.CNa * geom.area * geom.lcp * aero;
domega = (M_tvc + M_aero) / max(I, 1);
dtheta = omega - vx / r;
[ddelta, drate] = actuator_deriv(delta, rate, deltaCmd, geom);
dm = -gcmd.F / (gcmd.Isp * E.g0);

dY = [vx * E.Re / r; vh; aNG(1) - vx * vh / r; aNG(2) - gcmd.g + vx * vx / r; ...
    dm; dtheta; domega; ddelta; drate];

aux.F = gcmd.F;
aux.Fmax = gcmd.Fmax;
aux.q = q;
aux.alpha = alpha;
aux.gammaAir = gammaAir;
aux.thetaAbsCmd = gcmd.thetaAbs;
aux.aNG = aNG;
aux.mu = gcmd.F * arm / max(I, 1);
aux.I = I;
aux.arm = arm;
aux.throttle = gcmd.F / max(gcmd.Fmax, 1);
aux.qAlpha_kPa_deg = (q / 1e3) * rad2deg(alpha);
aux.lambda = q * geom.CNa * geom.area * geom.lcp / max(I, 1);
end


function gcmd = guidance_command(t, Y, S, E, gravMu, mode)
% Same thrust magnitude and the same steering target as starshipStack.
h = Y(2);
vx = Y(3);
vh = Y(4);
m = max(Y(5), 1);
r = E.Re + h;
g = gravMu / r^2;
[~, rho, aSound] = earthModel(h, E);
v = hypot(vx, vh);
switch mode
    case 'stack'
        M = v / max(aSound, 1);
        mb = S.bucketMach;
        bucket = sat01((M - mb(1)) / (mb(2) - mb(1))) * sat01((mb(4) - M) / (mb(4) - mb(3)));
        F = S.T1 * (1 - (1 - S.thrBucket) * bucket);
        Isp = S.Isp1vac - (S.Isp1vac - S.Isp1sl) * rho / E.rho0;
        if t < S.tKick || v == 0
            thetaAbs = 0;
        elseif t < S.tKick + S.kickDur
            thetaAbs = deg2rad(S.kickDeg);
        else
            thetaAbs = atan2(vx, vh);
        end
        Fmax = S.T1;
    case 'ship'
        F = min(S.T2, S.gLimit * E.g0 * m);
        Isp = S.Isp2;
        aT = F / m;
        aUp = S.wnG^2 * (S.hCoast - h) - 2 * S.zG * S.wnG * vh + g - vx^2 / r;
        aUp = min(max(aUp, -aT), aT);
        if aT > 1
            ux = sqrt(max(aT^2 - aUp^2, 0));
            thetaAbs = atan2(ux, aUp);
        elseif v > 1
            thetaAbs = atan2(vx, vh);
        else
            thetaAbs = 0;
        end
        Fmax = S.T2;
    otherwise
        error('ascent_dynamics:Mode', 'Unknown mode "%s".', mode);
end
gcmd.thetaAbs = thetaAbs;
gcmd.F = F;
gcmd.Isp = Isp;
gcmd.Fmax = Fmax;
gcmd.rho = rho;
gcmd.aSound = aSound;
gcmd.v = v;
gcmd.g = g;
gcmd.r = r;
end


function [I, arm] = inertia_arm(geom, m)
if isfield(geom, 'inertia') && ~isempty(geom.inertia)
    I = geom.inertia;
else
    I = m * geom.L^2 / 12;
end
if isfield(geom, 'arm') && ~isempty(geom.arm)
    arm = geom.arm;
else
    arm = geom.L / 2;
end
end


function [ddelta, drate] = actuator_deriv(delta, rate, deltaCmd, geom)
wn = geom.actWn;
zeta = geom.actZeta;
rMax = geom.gimbalRate;
dMax = geom.deltaMax;
if delta >= dMax && rate > 0
    rate = 0;
elseif delta <= -dMax && rate < 0
    rate = 0;
end
acc = wn^2 * (deltaCmd - delta) - 2 * zeta * wn * rate;
if rate >= rMax && acc > 0
    acc = 0;
elseif rate <= -rMax && acc < 0
    acc = 0;
end
ddelta = rate;
drate = acc;
end


function y = sat01(x)
y = min(max(x, 0), 1);
end

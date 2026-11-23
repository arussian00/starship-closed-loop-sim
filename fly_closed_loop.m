function closed = fly_closed_loop(nominal, V, D, windMode, steady, limits)
%FLY_CLOSED_LOOP  Integrate the ascent with the pitch controller in the loop.
%
%   closed = fly_closed_loop(nominal, V, D, windMode, steady)
%   closed = fly_closed_loop(..., limits)
%
%   The stack ascent and the ship ascent are fixed-step. Every control
%   period the discrete cascade reads delayed angle-of-attack and its rate,
%   and holds a gimbal command. Thrust direction is body attitude plus that
%   gimbal. Angle of attack adds normal force and the pitching moment.
%   Booster flip/boostback/catch and the ship's belly-flop use
%   simulateBooster from the closed-loop state. Those phases are the sim's
%   guidance: the pitch-TVC law does not run them (RCS, grid fins, belly-flop).
%
%   limits.maxTime stops the powered integration early (verification).
%   limits.includeShip and limits.includeReturn skip the later phases.

if nargin < 4 || isempty(windMode)
    windMode = 'none';
end
if nargin < 5 || isempty(steady)
    steady = 0;
end
if nargin < 6 || isempty(limits)
    limits = struct();
end
if ~isfield(limits, 'maxTime'), limits.maxTime = inf; end
if ~isfield(limits, 'includeShip'), limits.includeShip = true; end
if ~isfield(limits, 'includeReturn'), limits.includeReturn = true; end

cfg = closed_loop_config();
dt = 1 / cfg.controlHz;
S = nominal.S;
E = nominal.Pb;
gravMu = nominal.gravMu;
ext = closed_loop_extensions(V, nominal);

[wProbe, windLabel] = closed_loop_wind(V, 0, windMode, steady); %#ok<ASGLU>
fprintf('Closed loop at %.0f Hz. Wind: %s\n', cfg.controlHz, windLabel);

geomStack = pitch_geometry(V, D, 'stack');
[log, tSep, Ysep, stackNote, stackOk] = integrate_mode( ...
    'stack', zeros_state(nominal.m0), 0, nominal.mMECO, ...
    dt, limits.maxTime, S, E, gravMu, geomStack, D, V, ext, windMode, steady);

closed = blank_result(nominal, D, ext, windLabel, dt);
closed.stackOk = stackOk;
    closed.notes = [{stackNote}, ext.notes];
if ~stackOk
    closed.unavailable = true;
    closed.controllerNote = ['controller unavailable: stack ascent left the ' ...
        'pitch envelope (' stackNote ')'];
    closed.fullB = nominal.fullB;
    closed.fullS = nominal.fullS;
    closed.tSep = nominal.tSep;
    closed.tSECO = nominal.tSECO;
    closed.metricsB = nominal.metricsB;
    closed.metricsS = nominal.metricsS;
    closed.shipMode = 'guidance';
    closed.returnMode = 'guidance';
    return
end

closed.tSep = tSep;
closed.Ysep = Ysep;
closed.log = log;
logStack = log;

if ~limits.includeShip || tSep >= limits.maxTime - 1e-9
    closed.shipMode = 'not run';
    closed.returnMode = 'not run';
    closed.fullB = segment_only(log, log.t <= tSep + 1e-9, S.T1, nominal, 'stack');
    closed.fullS = closed.fullB;
    return
end

Yship = Ysep;
Yship(5) = nominal.mShip;
Yship(8) = 0;
Yship(9) = 0;
geomShip = pitch_geometry(V, D, 'ship');
[logShip, tSECO, Yseco, shipNote, shipOk] = integrate_mode( ...
    'ship', Yship, tSep, nominal.mMECO, ...
    dt, limits.maxTime, S, E, gravMu, geomShip, D, V, ext, windMode, steady);

if shipOk
    closed.shipMode = 'closed-loop';
    closed.tSECO = tSECO;
    closed.Yseco = Yseco;
    log = append_log(trim_log(log, log.t < tSep - 1e-9), logShip);
else
    closed.shipMode = 'guidance';
    closed.notes{end + 1} = ['ship ascent fell back to guidance (' shipNote ')']; %#ok<AGROW>
    [tC, YC, tSECO, Yseco] = ship_guidance(S, E, gravMu, Yship, tSep);
    closed.tSECO = tSECO;
    closed.Yseco = Yseco;
    log = trim_log(log, log.t <= tSep + 1e-9);
    closed.guidanceShip.t = tC;
    closed.guidanceShip.Y = YC;
end
closed.log = log;

if ~limits.includeReturn
    closed.returnMode = 'not run';
    return
end

closed.returnMode = 'guidance';
mBoost = Ysep(5) - nominal.mShip;
Xb0 = [Ysep(1:4); mBoost; Ysep(6); Ysep(7)];
fprintf('Booster return from the closed-loop staging state (sim guidance).\n');
outB = simulateBooster(nominal.Pb, Xb0, tSep, nominal.Pb.phaseList);

Ps = nominal.Ps;
Ps.m_dry = S.m2dry + S.mPay;
if strcmp(closed.shipMode, 'closed-loop')
    Xs0 = [Yseco(1:4); Yseco(5); Yseco(6); Yseco(7)];
else
    Xs0 = [Yseco(1:4); Yseco(5); atan2(Yseco(3), Yseco(4)); Yseco(3) / (E.Re + Yseco(2))];
end
fprintf('Ship re-entry from %s (sim guidance).\n', closed.shipMode);
outS = simulateBooster(Ps, Xs0, tSECO, Ps.phaseList);

segA = log_segment(logStack, true(numel(logStack.t), 1), S.T1);
if strcmp(closed.shipMode, 'closed-loop')
    segC = log_segment(log, log.t >= tSep - 1e-9, S.T2);
else
    segC = pointMassSegment(closed.guidanceShip.t, closed.guidanceShip.Y, E, ...
        @(t, Y) starshipStack(t, Y, S, E, gravMu, 'ship'), S.T2);
end
closed.fullB = fullFlight({segA}, {'stack ascent'}, outB);
closed.fullS = fullFlight({segA, segC}, {'stack ascent', 'ship burn'}, outS);
closed.outB = outB;
closed.outS = outS;
closed.metricsB = flightMetrics(closed.fullB, false);
closed.metricsS = flightMetrics(closed.fullS, false);
closed.Pb = nominal.Pb;
closed.Ps = Ps;
end


function [log, tEnd, Yend, note, ok] = integrate_mode( ...
    mode, Y, t, mStop, dt, maxTime, S, E, gravMu, geom0, D, V, ext, windMode, steady)
note = '';
ok = true;
nMax = ceil(max(maxTime - t, 0) / dt) + 5;
if isfinite(maxTime)
    nMax = min(nMax, ceil((maxTime - t) / dt) + 5);
else
    nMax = ceil(1600 / dt) + 5;
end
nMax = max(nMax, 2);
log = empty_log(nMax);
st = discrete_pitch_init(D, dt);
[alpha, alphaDot, aux] = measure(t, Y, S, E, gravMu, mode, geom0, ext, V, windMode, steady);
alphaCmd = cl_wrap(aux.thetaAbsCmd - aux.gammaAir);
st = discrete_pitch_align(st, alpha, alphaDot, alphaCmd);
reliefOn = ext.loadRelief;
reliefWarned = false;
hitEvent = false;

n = 1;
log = write_sample(log, n, t, Y, aux, alphaCmd, 0, true);
vPrev = event_value(mode, t, Y, mStop, S, E, gravMu);
tWall = tic;
nextPrint = t + 30;

while t < maxTime - 1e-12
    Yprev = Y;
    tPrev = t;
    geom = geom_at(geom0, ext, t);
    w = wind_scalar(V, t, windMode, steady);
    [alpha, alphaDot, aux] = measure(t, Y, S, E, gravMu, mode, geom, ext, V, windMode, steady);
    alphaCmd = cl_wrap(aux.thetaAbsCmd - aux.gammaAir);
    if reliefOn
        [alphaCmd, reliefOn, reliefWarned] = apply_relief( ...
            alphaCmd, alpha, aux.q, V, reliefOn, reliefWarned);
    end
    [st, deltaCmd] = discrete_pitch_step(st, alpha, alphaDot, Y(8), alphaCmd, aux.mu);
    Y = rk4_step(t, Y, dt, S, E, gravMu, mode, geom, deltaCmd, w);
    Y = project_actuator(Y, geom);
    t = t + dt;

    if ~all(isfinite(Y)) || Y(2) < -1 || abs(Y(6)) > 2.6
        ok = false;
        note = sprintf('diverged at t = %.2f s (h = %.1f m, theta = %.1f deg)', ...
            t, Y(2), rad2deg(Y(6)));
        fprintf('Closed-loop %s %s\n', mode, note);
        break
    end

    vNow = event_value(mode, t, Y, mStop, S, E, gravMu);
    [hit, frac] = event_hit(mode, vPrev, vNow);
    if hit
        hitEvent = true;
        Y = Yprev + frac * (Y - Yprev);
        t = tPrev + frac * dt;
        n = n + 1;
        [~, ~, aux] = measure(t, Y, S, E, gravMu, mode, geom, ext, V, windMode, steady);
        alphaCmd = cl_wrap(aux.thetaAbsCmd - aux.gammaAir);
        log = write_sample(log, n, t, Y, aux, alphaCmd, deltaCmd, true);
        break
    end
    vPrev = vNow;
    n = n + 1;
    [~, ~, aux] = measure(t, Y, S, E, gravMu, mode, geom, ext, V, windMode, steady);
    alphaCmd = cl_wrap(aux.thetaAbsCmd - aux.gammaAir);
    log = write_sample(log, n, t, Y, aux, alphaCmd, deltaCmd, true);

    if t >= nextPrint
        fprintf('  %s  t = %6.1f s   h = %7.2f km   alpha = %+6.2f deg   gimbal = %+5.2f deg\n', ...
            mode, t, Y(2) / 1e3, rad2deg(aux.alpha), rad2deg(Y(8)));
        nextPrint = t + 30;
    end
    if n >= nMax
        break
    end
end

if ok && ~hitEvent && ~(isfinite(maxTime) && t >= maxTime - 1.5 * dt)
    ok = false;
    if strcmp(mode, 'stack')
        note = 'stack did not reach hot staging';
    else
        note = 'ship burn did not reach SECO';
    end
    fprintf('Closed-loop %s %s\n', mode, note);
end

log = trim_log(log, (1:n).');
tEnd = t;
Yend = Y;
fprintf('  %s done at t = %.2f s in %.1f s wall time (%s)\n', ...
    mode, tEnd, toc(tWall), ternary(ok, 'in the envelope', note));
end


function [alpha, alphaDot, aux] = measure(t, Y, S, E, gravMu, mode, geom, ext, V, windMode, steady)
w = wind_scalar(V, t, windMode, steady);
[dY, aux] = ascent_dynamics(t, Y, S, E, gravMu, mode, geom, Y(8), w);
vrx = Y(3) - w;
vrh = Y(4);
vr2 = vrx * vrx + vrh * vrh;
if vr2 > 1
    gdot = (vrh * dY(3) - vrx * dY(4)) / vr2;
else
    gdot = 0;
end
alpha = aux.alpha;
alphaDot = (Y(7) - Y(3) / (E.Re + Y(2))) - gdot;
end


function Y = rk4_step(t, Y, dt, S, E, gravMu, mode, geom, deltaCmd, w)
k1 = ascent_dynamics(t, Y, S, E, gravMu, mode, geom, deltaCmd, w);
k2 = ascent_dynamics(t + 0.5 * dt, Y + 0.5 * dt * k1, S, E, gravMu, mode, geom, deltaCmd, w);
k3 = ascent_dynamics(t + 0.5 * dt, Y + 0.5 * dt * k2, S, E, gravMu, mode, geom, deltaCmd, w);
k4 = ascent_dynamics(t + dt, Y + dt * k3, S, E, gravMu, mode, geom, deltaCmd, w);
Y = Y + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
end


function Y = project_actuator(Y, geom)
rMax = geom.gimbalRate;
dMax = geom.deltaMax;
if Y(9) > rMax
    Y(9) = rMax;
elseif Y(9) < -rMax
    Y(9) = -rMax;
end
if Y(8) > dMax
    Y(8) = dMax;
    if Y(9) > 0
        Y(9) = 0;
    end
elseif Y(8) < -dMax
    Y(8) = -dMax;
    if Y(9) < 0
        Y(9) = 0;
    end
end
end


function Y0 = zeros_state(m0)
Y0 = [0; 0; 0; 0; m0; 0; 0; 0; 0];
end


function v = event_value(mode, t, Y, mStop, S, E, gravMu) %#ok<INUSD>
if strcmp(mode, 'stack')
    v = Y(5) - mStop;
else
    [v, ~, ~] = perigeeSECO(t, Y(1:5), S, E, gravMu);
end
end


function [hit, frac] = event_hit(mode, v0, v1)
hit = false;
frac = 1;
if strcmp(mode, 'stack')
    if v0 > 0 && v1 <= 0
        hit = true;
        frac = v0 / (v0 - v1);
    end
else
    if v0(1) < 0 && v1(1) >= 0
        hit = true;
        frac = -v0(1) / (v1(1) - v0(1));
    elseif v0(2) > 0 && v1(2) <= 0
        hit = true;
        frac = v0(2) / (v0(2) - v1(2));
    end
end
frac = cl_clip(frac, 0, 1);
end


function [tC, YC, tSECO, Yseco] = ship_guidance(S, E, gravMu, Yship, tSep)
Y0 = [Yship(1:4); Yship(5)];
opts = odeset('Events', @(t, Y) perigeeSECO(t, Y, S, E, gravMu), ...
    'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tC, YC] = ode45(@(t, Y) starshipStack(t, Y, S, E, gravMu, 'ship'), ...
    [tSep, tSep + 1500], Y0, opts);
tSECO = tC(end);
Yseco = YC(end, :).';
end


function geom = geom_at(geom, ext, t)
if strcmp(geom.config, 'ship') || ~ext.hasSchedule || isempty(ext.t)
    return
end
tt = cl_clip(t, ext.t(1), ext.t(end));
if ~isempty(ext.lcp)
    geom.lcp = interp1(ext.t, ext.lcp, tt, 'linear');
end
if ~isempty(ext.arm)
    geom.arm = interp1(ext.t, ext.arm, tt, 'linear');
end
if ~isempty(ext.inertia)
    geom.inertia = interp1(ext.t, ext.inertia, tt, 'linear');
end
end


function [alphaCmd, stillOn, warned] = apply_relief(alphaCmd, alpha, q, V, stillOn, warned)
try
    updated = load_relief_command(alphaCmd, alpha, q, V);
    if isnumeric(updated) && isscalar(updated) && isfinite(updated)
        alphaCmd = updated;
    end
catch ME
    if ~warned
        warning('fly_closed_loop:LoadRelief', ...
            'load_relief_command was not used (%s).', ME.message);
    end
    stillOn = false;
    warned = true;
end
end


function w = wind_scalar(V, t, windMode, steady)
[w, ~] = closed_loop_wind(V, t, windMode, steady);
w = w(1);
end


function log = empty_log(n)
z = nan(n, 1);
log.t = z;
log.x = z;
log.h = z;
log.vx = z;
log.vh = z;
log.m = z;
log.theta = z;
log.omega = z;
log.delta = z;
log.rate = z;
log.alpha = z;
log.alphaCmd = z;
log.thetaCmd = z;
log.q = z;
log.qAlpha = z;
log.mu = z;
log.F = z;
log.throttle = z;
log.gLoad = z;
log.flying = false(n, 1);
log.deltaCmd = z;
end


function log = write_sample(log, n, t, Y, aux, alphaCmd, deltaCmd, flying)
log.t(n) = t;
log.x(n) = Y(1);
log.h(n) = Y(2);
log.vx(n) = Y(3);
log.vh(n) = Y(4);
log.m(n) = Y(5);
log.theta(n) = Y(6);
log.omega(n) = Y(7);
log.delta(n) = Y(8);
log.rate(n) = Y(9);
log.alpha(n) = aux.alpha;
log.alphaCmd(n) = alphaCmd;
log.thetaCmd(n) = aux.thetaAbsCmd;
log.q(n) = aux.q;
log.qAlpha(n) = aux.qAlpha_kPa_deg;
log.mu(n) = aux.mu;
log.F(n) = aux.F;
log.throttle(n) = aux.throttle;
log.gLoad(n) = norm(aux.aNG) / 9.80665;
log.flying(n) = flying;
log.deltaCmd(n) = deltaCmd;
end


function log = trim_log(log, keep)
names = fieldnames(log);
for k = 1:numel(names)
    log.(names{k}) = log.(names{k})(keep);
end
end


function log = append_log(a, b)
names = fieldnames(a);
for k = 1:numel(names)
    log.(names{k}) = [a.(names{k}); b.(names{k})];
end
end


function seg = log_segment(log, mask, Fmax)
t = log.t(mask);
n = numel(t);
X = [log.x(mask), log.h(mask), log.vx(mask), log.vh(mask), log.m(mask), ...
    log.theta(mask), log.omega(mask)];
aux.throttle = log.throttle(mask);
aux.thrust = log.F(mask);
% Flame graphic uses the sim sign (thrust angle = theta - gimbal).
aux.gimbal = -log.delta(mask);
aux.gLoad = log.gLoad(mask);
aux.q = log.q(mask);
aux.thetaCmd = log.thetaCmd(mask);
aux.tauRCS = zeros(n, 1);
aux.finForce = zeros(n, 1);
aux.wind = zeros(n, 1);
if nargin >= 3 && ~isempty(Fmax) && Fmax > 0
    aux.throttle = aux.thrust / Fmax;
end
seg = struct('t', t, 'X', X, 'aux', aux);
end


function full = segment_only(log, mask, Fmax, nominal, ~)
seg = log_segment(log, mask, Fmax);
full = struct('t', seg.t, 'X', seg.X, 'aux', seg.aux, ...
    'phase', ones(numel(seg.t), 1), 'phaseList', {{'stack ascent'}}, ...
    'phaseStart', seg.t(1), 'P', nominal.Pb, 'crashed', false, ...
    'status', 'partial', 'notes', {{}});
end


function closed = blank_result(nominal, D, ext, windLabel, dt)
closed.unavailable = false;
closed.controllerNote = '';
closed.windLabel = windLabel;
closed.dt = dt;
closed.D = D;
closed.ext = ext;
closed.nominal = nominal;
closed.stackOk = false;
closed.shipMode = 'closed-loop';
closed.returnMode = 'guidance';
closed.log = [];
closed.tSep = NaN;
closed.tSECO = NaN;
closed.metricsB = [];
closed.metricsS = [];
closed.fullB = [];
closed.fullS = [];
closed.notes = ext.notes;
end


function label = ternary(flag, a, b)
if flag
    label = a;
else
    label = b;
end
end

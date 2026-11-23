function R = pitch_step_reference(D, mu, lambda, opts)
%PITCH_STEP_REFERENCE  Step response of the discrete law and of the diagram.
%
%   R = pitch_step_reference(D, mu, lambda)
%   R = pitch_step_reference(D, mu, lambda, opts)
%
%   The continuous trace integrates the boosterPitchTVC block diagram at
%   1 ms: prefilter, governor, cascade, anti-windup, 20 ms sensor delay,
%   second-order actuator with the rate and angle stops, and
%   theta'' = mu*sin(delta) + lambda*theta.
%   The discrete trace is discrete_pitch_step at the closed-loop rate, with
%   that same plant integrated at 1 ms under a held gimbal command.
%
%   When Simulink can build and run boosterPitchTVC, R.simulink is that
%   sim() result for this frozen mu and lambda. Otherwise
%   R.simulink.available is false and R.simulink.reason says why. Pass
%   opts.stubSimulink = true to force the catch without Simulink.

if nargin < 4 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'stepDeg'), opts.stepDeg = D.thetaStepDeg; end
if ~isfield(opts, 'stepTime'), opts.stepTime = D.thetaStepTime; end
if ~isfield(opts, 'tStop'), opts.tStop = D.tStop; end
if ~isfield(opts, 'dtFine'), opts.dtFine = 0.001; end
if ~isfield(opts, 'dtCtrl')
    opts.dtCtrl = 1 / closed_loop_config().controlHz;
end
if ~isfield(opts, 'stubSimulink'), opts.stubSimulink = false; end
if ~isfield(opts, 'trySimulink'), opts.trySimulink = true; end

R.continuous = integrate_diagram(D, mu, lambda, opts, 'continuous');
R.discrete = integrate_diagram(D, mu, lambda, opts, 'discrete');
R.errDeg = step_error(R.continuous, R.discrete);
R.simulink = simulink_step(D, mu, lambda, opts);
if R.simulink.available
    R.simulinkErrDeg = step_error(R.simulink, R.discrete);
else
    R.simulinkErrDeg = NaN;
end
end


function out = integrate_diagram(D, mu, lambda, opts, kind)
dt = opts.dtFine;
t = (0:dt:opts.tStop).';
n = numel(t);
theta = zeros(n, 1);
q = zeros(n, 1);
delta = zeros(n, 1);
stepRad = deg2rad(opts.stepDeg);
stAct = [0; 0];
stPlant = [0; 0];

if strcmp(kind, 'continuous')
    nDelay = max(1, round(D.sensorDelay / dt));
    bufT = zeros(nDelay, 1);
    bufQ = zeros(nDelay, 1);
    pref = 0;
    qf = 0;
    trim = 0;
    shaped = 0;
    integ = 0;
    for k = 1:n
        cmd = step_at(t(k), opts.stepTime, stepRad);
        thetaD = bufT(1);
        qD = bufQ(1);
        bufT = [bufT(2:end); stPlant(1)]; %#ok<AGROW>
        bufQ = [bufQ(2:end); stPlant(2)]; %#ok<AGROW>
        [pref, qf, trim, shaped, integ, deltaCmd] = control_euler( ...
            pref, qf, trim, shaped, integ, thetaD, qD, stAct(1), cmd, mu, D, dt);
        stAct = plant_substep(stAct, deltaCmd, D, dt, 'actuator');
        stPlant = plant_substep(stPlant, stAct(1), D, dt, 'rigid', mu, lambda);
        theta(k) = stPlant(1);
        q(k) = stPlant(2);
        delta(k) = stAct(1);
    end
else
    st = discrete_pitch_init(D, opts.dtCtrl);
    holdSteps = max(1, round(opts.dtCtrl / dt));
    deltaCmd = 0;
    for k = 1:n
        if mod(k - 1, holdSteps) == 0
            cmd = step_at(t(k), opts.stepTime, stepRad);
            [st, deltaCmd] = discrete_pitch_step(st, stPlant(1), stPlant(2), ...
                stAct(1), cmd, mu);
        end
        stAct = plant_substep(stAct, deltaCmd, D, dt, 'actuator');
        stPlant = plant_substep(stPlant, stAct(1), D, dt, 'rigid', mu, lambda);
        theta(k) = stPlant(1);
        q(k) = stPlant(2);
        delta(k) = stAct(1);
    end
end

out.t = t;
out.thetaDeg = rad2deg(theta);
out.qDeg = rad2deg(q);
out.deltaDeg = rad2deg(delta);
out.kind = kind;
out.finalDeg = out.thetaDeg(end);
out.overshootPct = max(0, (max(out.thetaDeg) - opts.stepDeg) / max(opts.stepDeg, eps) * 100);
end


function [pref, qf, trim, shaped, integ, deltaCmd] = control_euler( ...
    pref, qf, trim, shaped, integ, thetaD, qD, delta, cmd, mu, D, dt)
% One Euler step of the Simulink controller ODEs. At 1 ms this is the
% reference the 100 Hz law is compared against.
pref = pref + dt * (cmd - pref) / D.cmdTau;
qf = qf + dt * (qD - qf) / D.Tf;
trim = trim + dt * (delta - trim) / D.trimTau;
if mu < 1e-4
    deltaCmd = 0;
    return
end
avail = max(D.deltaMax - abs(trim), D.deltaAvailFloor);
aAvail = mu * sin(cl_clip(avail, 0, pi / 2));
qLim = max(D.shapeQ * aAvail, D.qLimFloor);
eLim = max(D.shapeE * aAvail, D.eLimFloor);
lead = cl_clip(pref - thetaD, -eLim, eLim);
target = thetaD + lead;
sdot = cl_clip(D.shapeKs * (target - shaped), -qLim, qLim);
shaped = shaped + dt * sdot;
e = shaped - thetaD;
qUnsat = D.Kth * e + integ;
qCmd = cl_clip(qUnsat, -qLim, qLim);
aRaw = D.Kq * (qCmd - qf);
aMax = mu * D.deltaMax;
aCmd = cl_clip(aRaw, -aMax, aMax);
deltaCmd = aCmd / mu;
iDot = D.KiTh * e + D.kaw * (qCmd - qUnsat) + (D.kaw / max(D.Kq, 1e-9)) * (aCmd - aRaw);
integ = integ + dt * iDot;
end


function x = plant_substep(x, u, D, dt, which, mu, lambda)
if nargin < 6
    mu = 0;
    lambda = 0;
end
k1 = plant_dot(x, u, D, which, mu, lambda);
k2 = plant_dot(x + 0.5 * dt * k1, u, D, which, mu, lambda);
k3 = plant_dot(x + 0.5 * dt * k2, u, D, which, mu, lambda);
k4 = plant_dot(x + dt * k3, u, D, which, mu, lambda);
x = x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
if strcmp(which, 'actuator')
    x = project_actuator(x, D);
end
end


function dx = plant_dot(x, u, D, which, mu, lambda)
if strcmp(which, 'actuator')
    delta = x(1);
    rate = x(2);
    if delta >= D.deltaMax && rate > 0
        rate = 0;
    elseif delta <= -D.deltaMax && rate < 0
        rate = 0;
    end
    acc = D.actWn^2 * (u - delta) - 2 * D.actZeta * D.actWn * rate;
    if rate >= D.gimbalRate && acc > 0
        acc = 0;
    elseif rate <= -D.gimbalRate && acc < 0
        acc = 0;
    end
    dx = [rate; acc];
else
    theta = x(1);
    q = x(2);
    dx = [q; mu * sin(u) + lambda * theta];
end
end


function x = project_actuator(x, D)
if x(2) > D.gimbalRate
    x(2) = D.gimbalRate;
elseif x(2) < -D.gimbalRate
    x(2) = -D.gimbalRate;
end
if x(1) > D.deltaMax
    x(1) = D.deltaMax;
    if x(2) > 0
        x(2) = 0;
    end
elseif x(1) < -D.deltaMax
    x(1) = -D.deltaMax;
    if x(2) < 0
        x(2) = 0;
    end
end
end


function cmd = step_at(t, stepTime, stepRad)
if t >= stepTime
    cmd = stepRad;
else
    cmd = 0;
end
end


function err = step_error(ref, other)
% ref and other share the 1 ms grid when both came from integrate_diagram.
% A Simulink log is interpolated onto that grid.
if numel(ref.t) == numel(other.t) && max(abs(ref.t - other.t)) < 1e-9
    y = other.thetaDeg;
else
    y = interp1(other.t, other.thetaDeg, ref.t, 'linear', 'extrap');
end
d = ref.thetaDeg - y;
err.maxAbs = max(abs(d));
err.rms = sqrt(mean(d .^ 2));
err.finalRef = ref.thetaDeg(end);
err.finalOther = y(end);
end


function R = simulink_step(D, mu, lambda, opts)
R.available = false;
R.reason = '';
R.t = [];
R.thetaDeg = [];
R.kind = 'simulink';
try
    if opts.stubSimulink
        error('pitch_step_reference:Stub', ...
            'Simulink stubbed: sim(''boosterPitchTVC'') was not called');
    end
    if ~opts.trySimulink
        R.reason = 'Simulink comparison turned off';
        return
    end
    if exist('sim', 'file') ~= 2
        R.reason = 'sim is not on the path';
        return
    end
    if exist('build_pitch_model', 'file') ~= 2 || exist('apply_pitch_condition', 'file') ~= 2
        R.reason = 'build_pitch_model or apply_pitch_condition is not on the path';
        return
    end
    build_pitch_model();
    Dsim = D;
    Dsim.flight = struct('name', "step", 'mu', mu, 'lambda', lambda);
    apply_pitch_condition(Dsim, "step", ...
        'thetaStepDeg', opts.stepDeg, ...
        'thetaStepTime', opts.stepTime, ...
        'gustSpeed', 0, ...
        'tStop', opts.tStop, ...
        'shapeOn', 1);
    out = sim('boosterPitchTVC', 'ReturnWorkspaceOutputs', 'on');
    [t, theta] = unpack_theta(out);
    R.available = true;
    R.reason = '';
    R.t = t;
    R.thetaDeg = rad2deg(theta);
    R.finalDeg = R.thetaDeg(end);
    R.overshootPct = max(0, (max(R.thetaDeg) - opts.stepDeg) / max(opts.stepDeg, eps) * 100);
    R.kind = 'simulink';
catch ME
    R.available = false;
    R.reason = flatten_message(ME.message);
end
end


function [t, y] = unpack_theta(out)
ts = out.theta;
if isa(ts, 'timeseries')
    t = ts.Time(:);
    y = ts.Data(:);
elseif isstruct(ts) && isfield(ts, 'time')
    t = ts.time(:);
    y = ts.signals.values(:);
else
    error('pitch_step_reference:Log', 'sim() did not return a theta log.');
end
end


function text = flatten_message(text)
if ~ischar(text)
    text = char(text);
end
text = strrep(text, sprintf('\n'), ' ');
text = strrep(text, sprintf('\r'), ' ');
text = regexprep(text, ' +', ' ');
text = strtrim(text);
end

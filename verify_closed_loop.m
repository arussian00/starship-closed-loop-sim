function verify_closed_loop(mode)
%VERIFY_CLOSED_LOOP  Headless checks for the discrete pitch loop.
%
%   verify_closed_loop          poles, step response, 20 s ascent, full flight
%   verify_closed_loop('core')  stop before the full mission
%
%   Simulink is used when sim() and build_pitch_model exist. Otherwise the
%   discrete controller is compared with a 1 ms integration of the same
%   block diagram, and a stub proves the sim() failure is caught.

if nargin < 1 || isempty(mode)
    mode = 'all';
end
mode = lower(char(mode));
here = fileparts(mfilename('fullpath'));
if isempty(here)
    here = pwd;
end
addpath(here);
rehash;
check_run_options();
setup_paths();

cfg = closed_loop_config();
V = vehicle_params();
D = pitch_law(V);
fprintf('\nPitch law source: %s\n', D.source);
fprintf('Gains: Kq %.6g, Kth %.6g, KiTh %.6g, Kp %.6g, Ki %.6g, Kd %.6g, Tf %.6g\n', ...
    D.Kq, D.Kth, D.KiTh, D.Kp, D.Ki, D.Kd, D.Tf);

expect_gains(D);
check_derivative_match();
check_gust_sign(V);
check_poles(D, V);
check_step(D, V);
check_simulink_stub(D, V);
check_short_ascent(V, D, cfg);

if strcmp(mode, 'core')
    fprintf('\nVERIFY core: done (full flight not run)\n');
    return
end
check_full_flight(V, D);
fprintf('\nVERIFY all: done\n');
end


function expect_gains(D)
% Formulas in design_pitch_controller when vehicle_params does not override.
Kp = 1;
Ki = 0.02;
Kd = 2;
if abs(D.Kp - Kp) > 1e-9 || abs(D.Ki - Ki) > 1e-9 || abs(D.Kd - Kd) > 1e-9
    error('verify_closed_loop:Gains', ...
        'Expected Kp=1, Ki=0.02, Kd=2 from vehicle_params. Got %.6g %.6g %.6g.', ...
        D.Kp, D.Ki, D.Kd);
end
fprintf('Gain check: Kp Ki Kd match the design defaults.\n');
end


function check_derivative_match()
[S, Pb, gravMu, ~] = mission_pack();
geom = unit_geom(S);
samples = [0; 30; 80];
for i = 1:numel(samples)
    ts = samples(i);
    Y5 = state_at(S, Pb, gravMu, ts);
    g = guidance_at(ts, Y5, S, Pb, gravMu);
    Y = [Y5; g; 0; 0; 0];
    dY = ascent_dynamics(ts, Y, S, Pb, gravMu, 'stack', geom, 0, 0);
    dRef = starshipStack(ts, Y5, S, Pb, gravMu, 'stack');
    err = max(abs(dY(1:5) - dRef));
    fprintf('RHS match at t = %5.1f s: max |dY(1:5) difference| = %.3e\n', ts, err);
    if ts == 0 || ts >= 20
        if err > 1e-6
            error('verify_closed_loop:RHS', ...
                'ascent_dynamics does not match starshipStack at t = %.1f s (%.3e).', ...
                ts, err);
        end
    end
end
end


function check_gust_sign(V)
% Positive controls gust reduces alpha. closed_loop_wind stores that as a headwind.
[S, Pb, gravMu, ~] = mission_pack();
geom = unit_geom(S);
Y = [0; 1000; 0; 200; 5e6; 0; 0; 0; 0];
[w, ~] = closed_loop_wind(V, V.gustTime, 'gust');
[~, aux0] = ascent_dynamics(V.gustTime, Y, S, Pb, gravMu, 'stack', geom, 0, 0);
[~, auxW] = ascent_dynamics(V.gustTime, Y, S, Pb, gravMu, 'stack', geom, 0, w);
fprintf('Gust sign: alpha still air %.4f deg, with design gust %.4f deg (w = %+.3f m/s)\n', ...
    rad2deg(aux0.alpha), rad2deg(auxW.alpha), w);
if ~(auxW.alpha < aux0.alpha)
    error('verify_closed_loop:Gust', ...
        'Design gust did not reduce alpha.');
end
end


function check_poles(D, V)
f0 = flight_named(V, 'liftoff');
fq = flight_named(V, 'max-q');
fprintf('\nDesign rigid-body poles (liftoff, lambda = 0):\n');
print_poles(rigid_poles(D, 0));
fprintf('Design rigid-body poles (max-q, lambda = %.6g):\n', fq.lambda);
print_poles(rigid_poles(D, fq.lambda));
fprintf('Cascade + rate filter + actuator poles (liftoff, mu = %.6g):\n', f0.mu);
pAct = actuator_poles(D, f0.mu, 0);
print_poles(pAct);
fprintf('Cascade + rate filter + actuator poles (max-q):\n');
pMq = actuator_poles(D, fq.mu, fq.lambda);
print_poles(pMq);
if any(real(rigid_poles(D, fq.lambda)) >= -1e-8) || any(real(pAct) >= -1e-8) || any(real(pMq) >= -1e-8)
    error('verify_closed_loop:Poles', 'A closed-loop pole is not in the left half-plane.');
end
fprintf('Pole check: all reported closed-loop poles have negative real parts.\n');
end


function check_step(D, V)
f0 = flight_named(V, 'liftoff');
fq = flight_named(V, 'max-q');
opts.trySimulink = true;
opts.stubSimulink = false;
fprintf('\nStep response, liftoff (mu = %.6g, lambda = 0):\n', f0.mu);
report_step(pitch_step_reference(D, f0.mu, 0, opts), 5);
fprintf('\nStep response, max-q (mu = %.6g, lambda = %.6g):\n', fq.mu, fq.lambda);
report_step(pitch_step_reference(D, fq.mu, fq.lambda, opts), 5);
end


function check_simulink_stub(D, V)
f0 = flight_named(V, 'liftoff');
opts.trySimulink = true;
opts.stubSimulink = true;
opts.stepDeg = 5;
R = pitch_step_reference(D, f0.mu, 0, opts);
fprintf('\nSimulink stub: available = %d, reason = %s\n', ...
    R.simulink.available, R.simulink.reason);
if R.simulink.available || isempty(R.simulink.reason)
    error('verify_closed_loop:Stub', 'The Simulink stub did not fail closed.');
end
fprintf('Simulink stub: sim() failure was caught. Discrete law still returned a step.\n');
end


function check_short_ascent(V, D, cfg)
[S, Pb, Ps, gravMu, m0] = mission_params();
nominal = struct('S', S, 'Pb', Pb, 'Ps', Ps, 'gravMu', gravMu, ...
    'm0', m0, 'mShip', S.m2dry + S.m2prop + S.mPay, ...
    'mMECO', m0 - (S.m1prop - S.reserve1), 'fullB', []);
limits.maxTime = 20;
limits.includeShip = false;
limits.includeReturn = false;
fprintf('\n20 s closed-loop ascent (still air):\n');
closed = fly_closed_loop(nominal, V, D, 'none', 0, limits);
h = closed.log.h(end);
alpha = rad2deg(max(abs(closed.log.alpha)));
gimbal = rad2deg(max(abs(closed.log.delta)));
fprintf('  h(20 s) = %.2f km, peak |alpha| = %.3f deg, peak |gimbal| = %.3f deg, rate = %.0f Hz\n', ...
    h / 1e3, alpha, gimbal, cfg.controlHz);
Y20 = state_at(S, Pb, gravMu, 20);
fprintf('  open-loop h(20 s) = %.2f km,  |difference| = %.1f m\n', ...
    Y20(2) / 1e3, abs(h - Y20(2)));
if h < 500 || alpha > 15 || gimbal > D.deltaMaxDeg + 0.1
    error('verify_closed_loop:Ascent', ...
        'The 20 s ascent did not stay on a small-alpha climb.');
end
end


function check_full_flight(V, D)
fprintf('\n========== full closed-loop mission ==========\n');
nominal = nominal_flight();
closed = fly_closed_loop(nominal, V, D, 'none', 0);
if closed.unavailable
    error('verify_closed_loop:Flight', '%s', closed.controllerNote);
end
if ~strcmp(closed.shipMode, 'closed-loop')
    error('verify_closed_loop:Ship', 'Ship ascent did not stay closed-loop (%s).', ...
        closed.shipMode);
end
log = closed.log;
flying = log.flying;
err = rad2deg(cl_wrap(log.thetaCmd(flying) - log.theta(flying)));
fprintf('Peak |tracking error| %.3f deg\n', max(abs(err)));
fprintf('Peak |gimbal| %.3f deg (limit %.0f)\n', ...
    rad2deg(max(abs(log.delta(flying)))), D.deltaMaxDeg);
fprintf('Peak |q*alpha| %.4g kPa deg\n', max(abs(log.qAlpha(flying))));
fprintf('Staging closed  t=%.2f s  h=%.2f km  v=%.1f m/s\n', ...
    closed.tSep, closed.Ysep(2) / 1e3, hypot(closed.Ysep(3), closed.Ysep(4)));
fprintf('Staging nominal t=%.2f s  h=%.2f km  v=%.1f m/s\n', ...
    nominal.tSep, nominal.Ysep(2) / 1e3, hypot(nominal.Ysep(3), nominal.Ysep(4)));
fprintf('Catch  closed %.3f m   open-loop %.3f m   reference %.2f m\n', ...
    closed.metricsB.miss, nominal.metricsB.miss, closed_loop_config().openLoopCatch_m);
fprintf('Splash closed %.3f m   open-loop %.3f m   reference %.2f m\n', ...
    closed.metricsS.miss, nominal.metricsS.miss, closed_loop_config().openLoopSplash_m);
end


function report_step(R, stepDeg)
fprintf('  continuous: final %.3f deg, overshoot %.2f %%\n', ...
    R.continuous.finalDeg, R.continuous.overshootPct);
fprintf('  discrete:   final %.3f deg, overshoot %.2f %%\n', ...
    R.discrete.finalDeg, R.discrete.overshootPct);
fprintf('  |discrete - continuous| max %.4f deg, rms %.4f deg\n', ...
    R.errDeg.maxAbs, R.errDeg.rms);
if R.simulink.available
    fprintf('  simulink:   final %.3f deg, |discrete - sim| max %.4f deg\n', ...
        R.simulink.finalDeg, R.simulinkErrDeg.maxAbs);
else
    fprintf('  simulink:   unavailable (%s)\n', R.simulink.reason);
end
if R.errDeg.maxAbs > 0.5
    error('verify_closed_loop:Step', ...
        'Discrete and continuous  %.0f deg steps differ by %.3f deg.', ...
        stepDeg, R.errDeg.maxAbs);
end
if abs(R.discrete.finalDeg - stepDeg) > 0.5
    error('verify_closed_loop:StepFinal', ...
        'Discrete step settled at %.3f deg, not %.0f.', R.discrete.finalDeg, stepDeg);
end
end


function poles = rigid_poles(D, lambda)
A = [0, 1, 0; lambda - D.Kp, -D.Kd, D.Ki; -1, 0, 0];
poles = eig(A);
end


function poles = actuator_poles(D, mu, lambda)
% theta, q, integ, q_filter, delta, rate. No delay, no prefilter, no governor.
A = zeros(6, 6);
A(1, 2) = 1;
A(2, 1) = lambda;
A(2, 5) = mu;
A(3, 1) = -D.KiTh;
A(4, 2) = 1 / D.Tf;
A(4, 4) = -1 / D.Tf;
k = D.actWn^2 * D.Kq / mu;
A(6, 1) = k * (-D.Kth);
A(6, 3) = k;
A(6, 4) = -k;
A(6, 5) = -D.actWn^2;
A(6, 6) = -2 * D.actZeta * D.actWn;
A(5, 6) = 1;
poles = eig(A);
end


function print_poles(poles)
poles = poles(:);
[~, order] = sort(real(poles));
poles = poles(order);
for k = 1:numel(poles)
    fprintf('  %+.6f  %+.6fj\n', real(poles(k)), imag(poles(k)));
end
end


function f = flight_named(V, name)
f = [];
for k = 1:numel(V.flight)
    if strcmp(char(V.flight(k).name), name)
        f = V.flight(k);
        return
    end
end
error('verify_closed_loop:FlightName', 'No flight named %s.', name);
end


function [S, Pb, gravMu, m0] = mission_pack()
[S, Pb, ~, gravMu, m0] = mission_params();
end


function geom = unit_geom(S)
V = vehicle_params();
D = pitch_law(V);
geom = pitch_geometry(V, D, 'stack');
geom.area = S.A;
end


function Y5 = state_at(S, Pb, gravMu, tQuery)
m0 = S.m1dry + S.m1prop + S.m2dry + S.m2prop + S.mPay;
if tQuery <= 0
    Y5 = [0; 0; 0; 0; m0];
    return
end
opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[~, Y] = ode45(@(t, YY) starshipStack(t, YY, S, Pb, gravMu, 'stack'), ...
    [0 tQuery], [0; 0; 0; 0; m0], opts);
Y5 = Y(end, :).';
end


function theta = guidance_at(t, Y5, S, Pb, gravMu)
Y = [Y5; 0; 0; 0; 0];
[~, aux] = ascent_dynamics(t, Y, S, Pb, gravMu, 'stack', unit_geom(S), 0, 0);
theta = aux.thetaAbsCmd;
end

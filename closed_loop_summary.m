function S = closed_loop_summary(closed, nominal)
%CLOSED_LOOP_SUMMARY  Print the closed-loop comparison and return it.
%
%   Catch and splashdown for the open-loop column are this run of
%   matlab-rocket-sim. The reference column is the published open-loop
%   pair (catch -0.08 m, splash -1.47 m).

cfg = closed_loop_config();
S = empty_summary();
S.wind = closed.windLabel;
S.unavailable = closed.unavailable;
S.controllerNote = closed.controllerNote;
S.shipMode = closed.shipMode;
S.returnMode = closed.returnMode;
S.openLoopCatch_m = nominal.metricsB.miss;
S.openLoopSplash_m = nominal.metricsS.miss;
S.referenceCatch_m = cfg.openLoopCatch_m;
S.referenceSplash_m = cfg.openLoopSplash_m;
S.nominalSep_s = nominal.tSep;
S.nominalSep_h_m = nominal.Ysep(2);
S.nominalSep_v_mps = hypot(nominal.Ysep(3), nominal.Ysep(4));
S.nominalSep_gamma_deg = atan2d(nominal.Ysep(3), nominal.Ysep(4));

if closed.unavailable || isempty(closed.metricsB)
    S.catchMiss_m = S.openLoopCatch_m;
    S.splashMiss_m = S.openLoopSplash_m;
    print_summary(S, closed);
    return
end

S.catchMiss_m = closed.metricsB.miss;
S.splashMiss_m = closed.metricsS.miss;
S.catchSuccess = closed.metricsB.success;
S.splashSuccess = closed.metricsS.success;
S.tSep_s = closed.tSep;
S.sep_h_m = closed.Ysep(2);
S.sep_v_mps = hypot(closed.Ysep(3), closed.Ysep(4));
S.sep_gamma_deg = atan2d(closed.Ysep(3), closed.Ysep(4));
S.tSECO_s = closed.tSECO;

if ~isempty(closed.log)
    log = closed.log;
    flying = log.flying;
    if ~any(flying)
        flying = true(size(log.t));
    end
    err = rad2deg(cl_wrap(log.thetaCmd(flying) - log.theta(flying)));
    S.peakTrack_deg = max(abs(err));
    S.peakGimbal_deg = rad2deg(max(abs(log.delta(flying))));
    S.peakGimbalRate_deg = rad2deg(max(abs(log.rate(flying))));
    S.peakAlpha_deg = rad2deg(max(abs(log.alpha(flying))));
    S.peakQAlpha_kPa_deg = max(abs(log.qAlpha(flying)));
    dt = median(diff(log.t(flying)));
    if ~isfinite(dt) || dt <= 0
        dt = closed.dt;
    end
    dmax = closed.D.deltaMax;
    rmax = closed.D.gimbalRate;
    S.tOnAngle_s = dt * nnz(abs(log.delta(flying)) >= 0.98 * dmax);
    S.tOnRate_s = dt * nnz(abs(log.rate(flying)) >= 0.98 * rmax);
    S.gimbalLimit_deg = rad2deg(dmax);
    S.rateLimit_deg = rad2deg(rmax);
end

if isfield(closed, 'ext')
    S.loadRelief = closed.ext.loadRelief;
    S.movingCg = closed.ext.movingCg;
    S.slosh = closed.ext.slosh;
    S.bending = closed.ext.bending;
    S.gridFins = closed.ext.gridFins;
    S.monteCarlo = closed.ext.monteCarlo;
end
print_summary(S, closed);
end


function S = empty_summary()
S.unavailable = false;
S.controllerNote = '';
S.wind = '';
S.shipMode = '';
S.returnMode = '';
S.peakTrack_deg = NaN;
S.peakGimbal_deg = NaN;
S.peakGimbalRate_deg = NaN;
S.peakAlpha_deg = NaN;
S.peakQAlpha_kPa_deg = NaN;
S.tOnAngle_s = NaN;
S.tOnRate_s = NaN;
S.gimbalLimit_deg = NaN;
S.rateLimit_deg = NaN;
S.tSep_s = NaN;
S.sep_h_m = NaN;
S.sep_v_mps = NaN;
S.sep_gamma_deg = NaN;
S.tSECO_s = NaN;
S.nominalSep_s = NaN;
S.nominalSep_h_m = NaN;
S.nominalSep_v_mps = NaN;
S.nominalSep_gamma_deg = NaN;
S.catchMiss_m = NaN;
S.splashMiss_m = NaN;
S.openLoopCatch_m = NaN;
S.openLoopSplash_m = NaN;
S.referenceCatch_m = NaN;
S.referenceSplash_m = NaN;
S.catchSuccess = false;
S.splashSuccess = false;
S.loadRelief = false;
S.movingCg = false;
S.slosh = false;
S.bending = false;
S.gridFins = false;
S.monteCarlo = false;
end


function print_summary(S, closed)
fprintf('\n=========== CLOSED LOOP (controller flying) ===========\n');
if S.unavailable
    fprintf('%s\n', S.controllerNote);
    fprintf('The flight below is the open-loop sim. The controller did not fly it.\n');
end
fprintf('Max |gimbal|              : %9.3f deg   (limit %.0f deg)\n', ...
    S.peakGimbal_deg, S.gimbalLimit_deg);
fprintf('Max |gimbal rate|         : %9.3f deg/s (limit %.0f deg/s)\n', ...
    S.peakGimbalRate_deg, S.rateLimit_deg);
fprintf('Time on the angle stops   : %9.3f s\n', S.tOnAngle_s);
fprintf('Time on the rate stops    : %9.3f s\n', S.tOnRate_s);
fprintf('Max |q*alpha|             : %9.4g kPa deg\n', S.peakQAlpha_kPa_deg);
fprintf('Peak |tracking error|     : %9.3f deg   (guidance pitch minus body pitch)\n', ...
    S.peakTrack_deg);
fprintf('Peak |AoA|                : %9.3f deg\n', S.peakAlpha_deg);
fprintf('Staging closed-loop       : t = %6.2f s   h = %7.2f km   v = %7.1f m/s   gamma = %5.2f deg\n', ...
    S.tSep_s, S.sep_h_m / 1e3, S.sep_v_mps, S.sep_gamma_deg);
fprintf('Staging nominal open-loop : t = %6.2f s   h = %7.2f km   v = %7.1f m/s   gamma = %5.2f deg\n', ...
    S.nominalSep_s, S.nominalSep_h_m / 1e3, S.nominalSep_v_mps, S.nominalSep_gamma_deg);
fprintf('Staging difference        : dt = %+6.2f s   dh = %+8.1f m   dv = %+6.2f m/s\n', ...
    S.tSep_s - S.nominalSep_s, S.sep_h_m - S.nominalSep_h_m, S.sep_v_mps - S.nominalSep_v_mps);
fprintf('Catch miss                : %9.3f m closed-loop    %9.3f m open-loop    %6.2f m reference\n', ...
    S.catchMiss_m, S.openLoopCatch_m, S.referenceCatch_m);
fprintf('Splashdown miss           : %9.3f m closed-loop    %9.3f m open-loop    %6.2f m reference\n', ...
    S.splashMiss_m, S.openLoopSplash_m, S.referenceSplash_m);
fprintf('Ship ascent               : %s\n', S.shipMode);
fprintf('Booster return and ship re-entry: %s (pitch TVC does not run RCS, grid fins, or belly-flop)\n', ...
    S.returnMode);
fprintf('Wind                      : %s\n', S.wind);
fprintf('Features: load relief %s, moving CG %s, slosh %s, bending %s, grid fins %s, Monte Carlo %s\n', ...
    on_off(S.loadRelief), on_off(S.movingCg), on_off(S.slosh), ...
    on_off(S.bending), on_off(S.gridFins), on_off(S.monteCarlo));
if isfield(closed, 'notes')
    for k = 1:numel(closed.notes)
        fprintf('  note: %s\n', closed.notes{k});
    end
end
fprintf('========================================================\n\n');
end


function label = on_off(flag)
if flag
    label = 'on';
else
    label = 'off';
end
end

function varargout = run_closed_loop(speed, showAnimation, wind)
%RUN_CLOSED_LOOP  Starship ascent flown by the pitch controller.
%
%   run_closed_loop
%   run_closed_loop(2)                 twice as fast
%   run_closed_loop(1, false)          summary only, no animation
%   run_closed_loop(1, true, 'gust')   design gust from vehicle_params
%   run_closed_loop(1, false, 'none')  still air (the default)
%   run_closed_loop(1, false, 10)      10 m/s steady downrange wind
%
%   MATLAB Online, from this folder, after a git checkout done with !git:
%
%       addpath(pwd)
%       rehash
%       clear functions
%       run_closed_loop
%
%   The discrete cascade is the boosterPitchTVC law (prefilter, command
%   governor, attitude and rate loops, anti-windup, delta = a_cmd/mu).
%   It runs at closed_loop_config.controlHz inside the ascent integrator.
%   Thrust points along body attitude plus gimbal. Angle of attack adds
%   lift and drag. Booster catch and ship splashdown after engine cutoff
%   use matlab-rocket-sim's own guidance, labeled on screen.
%
%   out = run_closed_loop(1, false) returns the histories and out.summary.

here = fileparts(mfilename('fullpath'));
if isempty(here)
    here = pwd;
end
addpath(here);
rehash;

% A missing input cannot be read. Fill those in before the call.
if nargin < 1, speed = []; end
if nargin < 2, showAnimation = []; end
if nargin < 3, wind = []; end
opts = parse_run_options(speed, showAnimation, wind);
if options_probe()
    % check_run_options sets this and stops after parsing, so the probe
    % does not fly or open a window.
    if nargout > 0
        varargout{1} = opts;
    end
    return
end
if opts.showAnimation
    present_figure('closeothers');
end
paths = setup_paths();

nominal = nominal_flight();
closed = [];
try
    V = vehicle_params();
    D = pitch_law(V);
    fprintf('Pitch law: %s\n', D.source);
    fprintf('Kq = %.6g, Kth = %.6g, KiTh = %.6g, Tf = %.6g s\n', ...
        D.Kq, D.Kth, D.KiTh, D.Tf);
    closed = fly_closed_loop(nominal, V, D, opts.windMode, opts.steady);
catch ME
    closed = controller_failed(nominal, ME.message);
    fprintf('\n%s\n', closed.controllerNote);
end

summary = closed_loop_summary(closed, nominal);
closed.summary = summary;
closed.paths = paths;

if opts.showAnimation
    present_figure('closeothers');
    animate_closed_loop(closed, nominal, opts.speed);
end
if nargout > 0
    result.paths = paths;
    result.nominal = nominal;
    result.closed = closed;
    result.summary = summary;
    varargout{1} = result;
end
end


function opts = parse_run_options(speed, showAnimation, wind)
opts.speed = 1;
opts.showAnimation = true;
opts.windMode = 'none';
opts.steady = 0;
if nargin >= 1 && ~isempty(speed)
    if isstruct(speed)
        if isfield(speed, 'speed'), opts.speed = speed.speed; end
        if isfield(speed, 'showAnimation'), opts.showAnimation = logical(speed.showAnimation); end
        if isfield(speed, 'wind'), opts = apply_wind(opts, speed.wind); end
        if isfield(speed, 'windMode'), opts.windMode = char(speed.windMode); end
        if isfield(speed, 'steady'), opts.steady = speed.steady; end
    else
        opts.speed = speed;
    end
end
if nargin >= 2 && ~isempty(showAnimation) && ~isstruct(speed)
    opts.showAnimation = logical(showAnimation);
end
if nargin >= 3 && ~isempty(wind) && ~isstruct(speed)
    opts = apply_wind(opts, wind);
end
if ~(isscalar(opts.speed) && isnumeric(opts.speed) && opts.speed > 0)
    error('run_closed_loop:Speed', 'speed must be a positive number.');
end
end


function opts = apply_wind(opts, wind)
if isstruct(wind)
    if isfield(wind, 'mode'), opts.windMode = char(wind.mode); end
    if isfield(wind, 'speed'), opts.steady = wind.speed; opts.windMode = 'steady'; end
    if isfield(wind, 'steady'), opts.steady = wind.steady; end
elseif isnumeric(wind)
    opts.windMode = 'steady';
    opts.steady = wind;
else
    opts.windMode = char(wind);
end
end


function closed = controller_failed(nominal, reason)
closed.unavailable = true;
closed.controllerNote = ['controller unavailable: ' flatten_message(reason)];
closed.windLabel = 'not applied';
closed.dt = NaN;
closed.D = struct('deltaMax', NaN, 'gimbalRate', NaN);
closed.ext = struct('loadRelief', false, 'movingCg', false, 'slosh', false, ...
    'bending', false, 'gridFins', false, 'monteCarlo', false, 'notes', {{}});
closed.nominal = nominal;
closed.stackOk = false;
closed.shipMode = 'guidance';
closed.returnMode = 'guidance';
closed.log = [];
closed.tSep = nominal.tSep;
closed.tSECO = nominal.tSECO;
closed.Ysep = nominal.Ysep;
closed.metricsB = nominal.metricsB;
closed.metricsS = nominal.metricsS;
closed.fullB = nominal.fullB;
closed.fullS = nominal.fullS;
closed.notes = {closed.controllerNote};
end


function tf = options_probe()
tf = false;
try
    tf = isappdata(0, 'ClosedLoopOptionsProbe') && getappdata(0, 'ClosedLoopOptionsProbe');
catch
    tf = false;
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

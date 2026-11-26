function check_run_options()
%CHECK_RUN_OPTIONS  Documented run_closed_loop forms must accept missing inputs.
%
%   check_run_options
%
%   Calls run_closed_loop(), run_closed_loop(2), run_closed_loop(2, false),
%   run_closed_loop(1, false), and the wind forms with showAnimation false.
%   A root flag makes run_closed_loop return the parsed options, so this
%   does not integrate a flight or open the mission-control window.

here = fileparts(mfilename('fullpath'));
if isempty(here)
    here = pwd;
end
addpath(here);
set_probe(true);
try
    expect(@() run_closed_loop(), 1, true, 'none', 0);
    expect(@() run_closed_loop(2), 2, true, 'none', 0);
    expect(@() run_closed_loop(2, false), 2, false, 'none', 0);
    expect(@() run_closed_loop(1, false), 1, false, 'none', 0);
    expect(@() run_closed_loop(1, false, 'none'), 1, false, 'none', 0);
    expect(@() run_closed_loop(1, false, 'gust'), 1, false, 'gust', 0);
    expect(@() run_closed_loop(1, false, 'playback'), 1, false, 'playback', 0);
    expect(@() run_closed_loop(1, false, 10), 1, false, 'steady', 10);
    expect_speed_rejected();
catch ME
    set_probe(false);
    rethrow(ME);
end
set_probe(false);
fprintf('check_run_options: documented forms parse.\n');
end


function expect(fn, speed, showAnimation, windMode, steady)
opts = fn();
if ~isstruct(opts) || ~isfield(opts, 'speed') || ~isfield(opts, 'windMode') ...
        || ~isfield(opts, 'showAnimation') || ~isfield(opts, 'steady')
    error('check_run_options:Probe', ...
        'run_closed_loop did not return parsed options.');
end
gotWind = char(opts.windMode);
if abs(double(opts.speed) - speed) > 1e-12 ...
        || logical(opts.showAnimation) ~= logical(showAnimation) ...
        || ~strcmp(gotWind, windMode) ...
        || abs(double(opts.steady) - steady) > 1e-12
    error('check_run_options:Form', ...
        ['Expected speed %g, animation %d, wind %s, steady %g. ' ...
         'Got speed %g, animation %d, wind %s, steady %g.'], ...
        speed, logical(showAnimation), windMode, steady, ...
        double(opts.speed), logical(opts.showAnimation), gotWind, double(opts.steady));
end
end


function expect_speed_rejected()
rejected = false;
try
    run_closed_loop(0, false);
catch ME
    rejected = strcmp(ME.identifier, 'run_closed_loop:Speed');
    if ~rejected
        rethrow(ME);
    end
end
if ~rejected
    error('check_run_options:Speed', 'speed 0 was accepted.');
end
end


function set_probe(flag)
try
    setappdata(0, 'ClosedLoopOptionsProbe', logical(flag));
catch ME
    if flag
        error('check_run_options:Probe', 'Cannot set the options probe (%s).', ME.message);
    end
end
end

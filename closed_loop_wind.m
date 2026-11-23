function [w, label] = closed_loop_wind(V, t, mode, steady)
%CLOSED_LOOP_WIND  Horizontal wind for the closed-loop ascent.
%
%   [w, label] = closed_loop_wind(V, t, mode)
%   [w, label] = closed_loop_wind(V, t, mode, steady)
%
%   w is positive downrange, the same sign as matlab-rocket-sim P.wind.
%   It changes the air-relative velocity, so it changes dynamic pressure,
%   drag, and angle of attack.
%
%   mode 'none'    still air.
%   mode 'gust'    the vehicle_params design gust (15 m/s at t = 14 s on
%                  main). A positive controls-repo gust reduces alpha, so
%                  it is applied as a headwind (w negative).
%   mode 'playback'  playback_wind when that function is on the path,
%                  otherwise the same profile: load_relief_wind when its
%                  fields exist, else the design gust. The sign conversion
%                  is the same as 'gust'.
%   mode 'steady'  a constant downrange wind of `steady` m/s (sim sign).

if nargin < 3 || isempty(mode)
    mode = 'none';
end
if nargin < 4 || isempty(steady)
    steady = 0;
end
mode = lower(strtrim(char(mode)));
t = t(:);

switch mode
    case 'none'
        w = zeros(size(t));
        label = 'still air';
    case 'steady'
        w = steady * ones(size(t));
        label = sprintf('steady wind %+.4g m/s downrange (sim sign)', steady);
    case {'gust', 'playback'}
        [wCtrl, label] = controls_gust(V, t, strcmp(mode, 'playback'));
        w = -wCtrl;
        label = ['headwind for controls gust (' label ')'];
    otherwise
        error('closed_loop_wind:Mode', ...
            'wind mode must be none, gust, playback, or steady.');
end
end


function [w, label] = controls_gust(V, t, preferPlayback)
w = zeros(size(t));
label = 'no gust';
if preferPlayback && exist('playback_wind', 'file') == 2
    try
        [w, label] = playback_wind(V, t);
        w = w(:);
        if numel(w) == numel(t) && isnumeric(w)
            label = ['playback_wind: ' char(label)];
            return
        end
    catch ME
        warning('closed_loop_wind:Playback', ...
            'playback_wind was not used (%s).', ME.message);
    end
end

windFields = {'windShear', 'windShearTime', 'windGust', 'windGustStart', 'windGustDur'};
if exist('load_relief_wind', 'file') == 2 && has_fields(V, windFields)
    try
        w = load_relief_wind(V, t);
        w = w(:);
        if numel(w) == numel(t) && isnumeric(w)
            label = 'load_relief_wind (shear + 1-cosine gust)';
            return
        end
    catch ME
        warning('closed_loop_wind:LoadRelief', ...
            'load_relief_wind was not used (%s).', ME.message);
    end
end

w = zeros(size(t));
if isfield(V, 'gustTime') && isfield(V, 'gustSpeed')
    w(t >= V.gustTime) = V.gustSpeed;
    label = sprintf('design gust step: %.4g m/s at t = %.4g s (vehicle_params)', ...
        V.gustSpeed, V.gustTime);
else
    label = 'no controls-repo wind model found; wind left at 0';
end
end


function tf = has_fields(V, names)
tf = isstruct(V);
if ~tf
    return
end
for k = 1:numel(names)
    if ~isfield(V, names{k}) || isempty(V.(names{k}))
        tf = false;
        return
    end
end
end

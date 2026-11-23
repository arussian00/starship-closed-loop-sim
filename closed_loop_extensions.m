function ext = closed_loop_extensions(V, nominal)
%CLOSED_LOOP_EXTENSIONS  Steps 3-6, only when the function and fields exist.
%
%   ext = closed_loop_extensions(V, nominal)
%
%   The discrete law on main is the step 1-2 cascade. Later controls
%   branches add load relief, a moving CG, slosh, bending, grid fins, and
%   Monte Carlo. Each hook runs only when its function is on the path and
%   the field it reads exists. A missing piece is a note. Nothing here
%   edits the controls repo, and Monte Carlo is not launched.

ext = empty_ext();
traj = [];
if nargin >= 2 && ~isempty(nominal) && isfield(nominal, 'fullB') && ~isempty(nominal.fullB)
    traj = ascent_table(nominal, V);
end

ext = hook_load_relief(ext, V);
ext = hook_mass(ext, traj, V);
[ext, traj] = hook_named(ext, traj, V, {'apply_slosh', 'slosh_schedule'}, 'slosh');
[ext, traj] = hook_named(ext, traj, V, {'apply_bending', 'bending_schedule'}, 'bending');
if ~ext.slosh && exist('slosh_model', 'file') == 2
    ext.notes{end + 1} = ['slosh_model is on the path; the closed-loop body ' ...
        'calls it only through apply_slosh or slosh_schedule.']; %#ok<AGROW>
end
if ~ext.bending && exist('bending_model', 'file') == 2
    ext.notes{end + 1} = ['bending_model is on the path; the rigid closed-loop ' ...
        'body does not add bending states unless apply_bending returns a schedule.']; %#ok<AGROW>
end
ext = hook_fins(ext, traj, V);
ext = hook_monte_carlo(ext);
ext = schedule_from_traj(ext, traj);
end


function ext = empty_ext()
ext.loadRelief = false;
ext.movingCg = false;
ext.slosh = false;
ext.bending = false;
ext.gridFins = false;
ext.monteCarlo = false;
ext.notes = {};
ext.hasSchedule = false;
ext.t = [];
ext.arm = [];
ext.lcp = [];
ext.inertia = [];
ext.gridFinT = [];
ext.gridFinDeg = [];
end


function ext = hook_load_relief(ext, V)
hasFn = exist('load_relief_wind', 'file') == 2;
hasBlend = isfield(V, 'lrBlend') && ~isempty(V.lrBlend);
if hasFn && hasBlend && exist('load_relief_command', 'file') == 2
    ext.loadRelief = true;
    ext.notes{end + 1} = 'load_relief_command is on the path and will be called.'; %#ok<AGROW>
elseif hasFn && ~hasBlend
    ext.notes{end + 1} = ['load_relief_wind is on the path; lrBlend is not a ' ...
        'field, so the load-relief blend stays off. Wind can still be selected ' ...
        'with run_closed_loop(..., ''playback'').']; %#ok<AGROW>
elseif hasFn && hasBlend
    ext.notes{end + 1} = ['lrBlend is present but load_relief_command is not, ' ...
        'so the discrete law does not add a load-relief term.']; %#ok<AGROW>
end
end


function ext = hook_mass(ext, traj, V)
if isempty(traj)
    return
end
if exist('apply_mass_properties', 'file') == 2
    if isfield(V, 'staticMarginAscent') && isfield(V, 'staticMarginLanding')
        [traj2, ok, note] = try_call('apply_mass_properties', traj, V);
        if ok
            ext.movingCg = true;
            ext = schedule_from_traj(ext, traj2);
        end
        if ~isempty(note)
            ext.notes{end + 1} = note; %#ok<AGROW>
        end
    else
        ext.notes{end + 1} = ['apply_mass_properties is on the path; skipped ' ...
            'because staticMarginAscent or staticMarginLanding is missing.']; %#ok<AGROW>
    end
    return
end
[ext, ~] = hook_named(ext, traj, V, {'apply_cg_schedule', 'cg_schedule'}, 'movingCg');
end


function [ext, traj] = hook_named(ext, traj, V, names, flag)
for k = 1:numel(names)
    if exist(names{k}, 'file') ~= 2
        continue
    end
    [updated, ok, note] = try_call(names{k}, traj, V);
    if ok
        traj = updated;
        ext.(flag) = true;
        ext = schedule_from_traj(ext, traj);
        ext.notes{end + 1} = ['called ' names{k}]; %#ok<AGROW>
    elseif ~isempty(note)
        ext.notes{end + 1} = note; %#ok<AGROW>
    end
    return
end
end


function ext = hook_fins(ext, traj, V)
if exist('grid_fin_command', 'file') ~= 2
    if exist('grid_fin_model', 'file') == 2
        ext.notes{end + 1} = ['grid_fin_model is on the path; the return still ' ...
            'uses the sim grid fins. grid_fin_command is not on the path.']; %#ok<AGROW>
    end
    return
end
try
    fin = grid_fin_command(traj, V);
catch ME
    ext.notes{end + 1} = ['grid_fin_command failed: ' flatten_message(ME.message)]; %#ok<AGROW>
    return
end
[tFin, deg] = normalize_fin(fin);
if ~isempty(tFin)
    ext.gridFins = true;
    ext.gridFinT = tFin;
    ext.gridFinDeg = deg;
    ext.notes{end + 1} = 'called grid_fin_command (display only; descent uses the sim).'; %#ok<AGROW>
else
    ext.notes{end + 1} = 'grid_fin_command returned no t/deg trace.'; %#ok<AGROW>
end
end


function ext = hook_monte_carlo(ext)
names = {'run_pitch_monte_carlo', 'pitch_monte_carlo', 'run_monte_carlo'};
for k = 1:numel(names)
    if exist(names{k}, 'file') == 2
        ext.monteCarlo = true;
        ext.notes{end + 1} = [names{k} ' is on the path. The closed-loop run ' ...
            'does not launch it.']; %#ok<AGROW>
        return
    end
end
end


function [out, ok, note] = try_call(name, traj, V)
ok = false;
note = '';
out = traj;
try
    updated = feval(name, traj, V);
catch ME
    note = [name ' failed: ' flatten_message(ME.message)];
    return
end
if isstruct(updated) || (exist('istable', 'file') == 2 && istable(updated))
    out = updated;
    ok = true;
else
    note = [name ' returned no trajectory; the rod schedule stands.'];
end
end


function ext = schedule_from_traj(ext, traj)
[t, arm, lcp, inertia] = columns_of(traj);
if isempty(t)
    return
end
ext.hasSchedule = true;
ext.t = t;
ext.arm = arm;
ext.lcp = lcp;
ext.inertia = inertia;
end


function [t, arm, lcp, inertia] = columns_of(traj)
t = [];
arm = [];
lcp = [];
inertia = [];
if isempty(traj)
    return
end
t = column(traj, {'t_s', 't', 'time'});
arm = column(traj, {'arm_m', 'arm', 'gimbal_arm_m'});
lcp = column(traj, {'lcp_m', 'lcp'});
inertia = column(traj, {'inertia_kgm2', 'inertia'});
if isempty(t)
    t = [];
    arm = [];
    lcp = [];
    inertia = [];
end
end


function y = column(traj, names)
y = [];
for k = 1:numel(names)
    name = names{k};
    if isstruct(traj) && isfield(traj, name)
        y = traj.(name);
        y = y(:);
        return
    end
    if exist('istable', 'file') == 2 && istable(traj) && ismember(name, traj.Properties.VariableNames)
        y = traj.(name);
        y = y(:);
        return
    end
end
end


function traj = ascent_table(nominal, V)
fullB = nominal.fullB;
ascent = fullB.phase == 1;
t = fullB.t(ascent);
t = t(:);
X = fullB.X(ascent, :);
[t, ia] = unique(t, 'stable');
X = X(ia, :);
q = fullB.aux.q(ascent);
q = q(ia);
thrust = fullB.aux.thrust(ascent);
thrust = thrust(ia);
mass = X(:, 5);
h = X(:, 2);
L = V.stackLength;
arm = (L / 2) * ones(size(t));
lcp = (-V.staticMarginAscent * V.diam) * ones(size(t));
inertia = mass .* (L ^ 2) / 12;
traj = struct('t_s', t, 'altitude_m', h, 'mass_kg', mass, 'q_Pa', q(:), ...
    'thrust_N', thrust(:), 'arm_m', arm, 'lcp_m', lcp, 'inertia_kgm2', inertia, ...
    'length_m', L * ones(size(t)));
if exist('table', 'file') == 2 && exist('struct2table', 'file') == 2
    try
        traj = struct2table(traj);
    catch
    end
end
end


function [tFin, deg] = normalize_fin(fin)
tFin = [];
deg = [];
if ~isstruct(fin)
    return
end
if isfield(fin, 't')
    tFin = fin.t(:);
elseif isfield(fin, 'time')
    tFin = fin.time(:);
end
if isfield(fin, 'deg')
    deg = fin.deg(:);
elseif isfield(fin, 'deflection_deg')
    deg = fin.deflection_deg(:);
end
if numel(tFin) ~= numel(deg) || isempty(tFin)
    tFin = [];
    deg = [];
end
end


function text = flatten_message(text)
if ~ischar(text)
    text = char(text);
end
text = strrep(text, sprintf('\n'), ' ');
text = regexprep(text, ' +', ' ');
text = strtrim(text);
end

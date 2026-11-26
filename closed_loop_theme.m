function varargout = closed_loop_theme(action, varargin)
%CLOSED_LOOP_THEME  Colors for the mission-control window.
%
%   T = closed_loop_theme
%   closed_loop_theme('figure', fig)       white figure, light Theme when present
%   closed_loop_theme('graph', ax)         dark plot area, contrasting labels
%   closed_loop_theme('overlay', ax)       leave the plot color, contrasting labels
%   closed_loop_theme('refresh', fig)      choose text colors again after a redraw
%   closed_loop_theme('reinforce', fig)    same walk, used after a resize
%   rgb = closed_loop_theme('background', h)
%   ratio = closed_loop_theme('ratio', c1, c2)
%
%   Graph interiors stay the dark grey in T.plot. Line colors are not set
%   here. Gridlines stay the same grey. The figure margin is white, so
%   titles, axis labels, and outward ticks that sit on it are near-black.
%   Text drawn on a dark plot is near-white (#F0F0F0). Light panels, tab
%   strips, controls, and the banner keep near-black type. The ink is the
%   one with the higher WCAG contrast against the background the glyphs
%   actually use, and it is applied again on every refresh. A background
%   that cannot clear 4.5 with either ink (the camera sky in the middle of
%   its fade) gets a solid plate behind that text. Legends are left exactly
%   as the caller styled them.

if nargin == 0
    varargout{1} = theme_struct();
    return
end
switch lower(strtrim(char(action)))
    case 'figure'
        apply_figure(varargin{1});
    case 'graph'
        style_axes(varargin{1}, true);
    case 'overlay'
        style_axes(varargin{1}, false);
    case {'refresh', 'reinforce'}
        refresh_figure(varargin{1});
    case 'background'
        varargout{1} = glyph_background(varargin{1});
    case 'ratio'
        varargout{1} = contrast_ratio(varargin{1}, varargin{2});
    otherwise
        error('closed_loop_theme:Action', 'Unknown action "%s".', char(action));
end
end


function T = theme_struct()
T.figure = [1 1 1];
T.plot = [0.16 0.16 0.18];
T.darkText = [0.1 0.1 0.1];
T.lightText = [240 240 240] / 255;
T.text = T.darkText;
T.inPlot = T.lightText;
T.minContrast = 4.5;
T.grid = [0.62 0.63 0.66];
T.gridAlpha = 0.4;
T.bannerBack = [1 0.96 0.93];
T.legendText = [0.1 0.1 0.1];
T.legendBack = [1 1 1];
T.legendEdge = [0.1 0.1 0.1];
T.fontAxes = 10;
T.fontSmall = 9;
T.fontTitle = 11;
T.fontBanner = 13;
T.fontHud = 10;
end


function apply_figure(fig)
T = theme_struct();
try_set(fig, 'Color', T.figure);
if ~try_set(fig, 'Theme', 'light')
    try
        theme(fig, 'light');
    catch
    end
end
try_set(fig, 'Color', T.figure);
ink = choose_ink(T.figure);
try_set(fig, 'DefaultAxesXColor', ink);
try_set(fig, 'DefaultAxesYColor', ink);
try_set(fig, 'DefaultAxesZColor', ink);
try_set(fig, 'DefaultTextColor', ink);
try_set(fig, 'DefaultAxesFontSize', T.fontAxes);
try_set(fig, 'DefaultTextFontSize', T.fontSmall);
refresh_figure(fig);
end


function style_axes(ax, darkPlot)
if isempty(ax) || ~ishghandle(ax) || is_under_legend(ax)
    return
end
T = theme_struct();
if darkPlot
    paint_plot(ax, T);
    try
        setappdata(ax, 'ClosedLoopGraph', true);
    catch
    end
    partner = [];
    try
        partner = getappdata(ax, 'yyRight');
    catch
    end
    if ~isempty(partner) && ishghandle(partner) && partner ~= ax
        % Octave's yyaxis overlay stays transparent so it does not cover
        % the dark plot underneath. Its tick labels still take the ink
        % that contrasts with the background they sit on.
        apply_ink(partner);
    end
end
apply_ink(ax);
paint_label_fonts(ax, true);
end


function paint_plot(ax, T)
try_set(ax, 'Color', T.plot);
try_set(ax, 'GridColor', T.grid);
try_set(ax, 'MinorGridColor', T.grid);
try_set(ax, 'GridAlpha', T.gridAlpha);
try_set(ax, 'MinorGridAlpha', T.gridAlpha * 0.6);
try_set(ax, 'Box', 'on');
try_set(ax, 'TickDir', 'out');
end


function paint_label_fonts(ax, setFont)
T = theme_struct();
if setFont
    try_set(ax, 'FontSize', T.fontAxes);
end
names = {'Title', 'XLabel', 'YLabel', 'ZLabel'};
for n = 1:numel(names)
    h = safe_get(ax, names{n});
    if isempty(h) || ~ishghandle(h)
        continue
    end
    try_set(h, 'FontWeight', 'bold');
    if ~setFont
        continue
    end
    if strcmp(names{n}, 'Title')
        try_set(h, 'FontSize', T.fontTitle);
    else
        try_set(h, 'FontSize', T.fontAxes);
    end
end
end


function refresh_figure(fig)
if isempty(fig) || ~ishghandle(fig)
    return
end
T = theme_struct();
savedLegends = snapshot_legends(fig);
try_set(fig, 'Color', T.figure);
try
    objs = findall(fig);
catch
    objs = [];
end
for i = 1:numel(objs)
    restore_graph(objs(i), T);
end
for i = 1:numel(objs)
    apply_ink(objs(i));
end
restore_legends(savedLegends);
end


function restore_graph(h, T)
if isempty(h) || ~ishghandle(h) || is_under_legend(h)
    return
end
typ = object_type(h);
if ~strcmp(typ, 'axes')
    return
end
dark = false;
try
    dark = isappdata(h, 'ClosedLoopGraph') && getappdata(h, 'ClosedLoopGraph');
catch
    dark = false;
end
if dark
    paint_plot(h, T);
end
end


function apply_ink(h)
if isempty(h) || ~ishghandle(h) || is_under_legend(h)
    return
end
typ = object_type(h);
switch typ
    case 'axes'
        % Outward ticks and the box edge sit on the figure or panel, not
        % on the plot interior. 'in' would put the numbers on the plot.
        try_set(h, 'TickDir', 'out');
        ink = choose_ink(tick_scene(h));
        try_set(h, 'XColor', ink);
        try_set(h, 'YColor', ink);
        try_set(h, 'ZColor', ink);
        paint_rulers(h, ink);
    case {'text', 'textbox', 'textboxshape'}
        apply_text_ink(h);
    case 'hggroup'
        % Octave annotation textboxes are hggroup objects with a String.
        % Series groups have no String; their Color is the line color.
        if has_prop(h, 'String')
            apply_text_ink(h);
        end
    case 'colorbar'
        ink = choose_ink(container_background(h));
        try_set(h, 'Color', ink);
        apply_text_ink(safe_get(h, 'Label'));
        apply_text_ink(safe_get(h, 'Title'));
    case {'uicontrol', 'uipanel', 'uitab', 'uibuttongroup', 'uitable'}
        ink = choose_ink(own_or_container(h));
        try_set(h, 'ForegroundColor', ink);
        if has_prop(h, 'FontColor')
            try_set(h, 'FontColor', ink);
        end
    case {'textarrow', 'textarrowshape'}
        apply_text_ink(h);
end
end


function apply_text_ink(h)
if isempty(h) || ~ishghandle(h) || is_under_legend(h)
    return
end
[bg, userOwned] = ink_background(h);
[ink, plate] = choose_ink_plate(bg);
if ~isempty(plate) && ~userOwned && has_prop(h, 'BackgroundColor')
    try_set(h, 'BackgroundColor', plate);
    set_plate_flag(h, true);
elseif is_our_plate(h)
    try_set(h, 'BackgroundColor', 'none');
    set_plate_flag(h, false);
end
try_set(h, 'Color', ink);
if has_prop(h, 'TextColor')
    try_set(h, 'TextColor', ink);
end
if has_prop(h, 'FontColor')
    try_set(h, 'FontColor', ink);
end
end


function paint_rulers(ax, ink)
names = {'XAxis', 'YAxis', 'ZAxis'};
for n = 1:numel(names)
    rulers = safe_get(ax, names{n});
    for k = 1:numel(rulers)
        try_set(rulers(k), 'Color', ink);
    end
end
end


function rgb = glyph_background(h)
% Color directly behind the glyphs, including a plate this theme added.
if isempty(h) || ~ishghandle(h)
    rgb = [1 1 1];
    return
end
if is_under_legend(h)
    rgb = legend_box(h);
    return
end
own = color_rgb(safe_get(h, 'BackgroundColor'));
if isempty(own) && strcmp(object_type(h), 'legend')
    own = color_rgb(safe_get(h, 'Color'));
end
if ~isempty(own)
    rgb = own;
    return
end
rgb = underlying_scene(h);
end


function [bg, userOwned] = ink_background(h)
% Background the ink is chosen against. A plate we added is ignored so a
% later redraw can take it away when the scene itself is readable.
userOwned = false;
if is_our_plate(h)
    bg = underlying_scene(h);
    return
end
own = color_rgb(safe_get(h, 'BackgroundColor'));
if ~isempty(own)
    bg = own;
    userOwned = true;
    return
end
bg = underlying_scene(h);
end


function rgb = underlying_scene(h)
rgb = [1 1 1];
if isempty(h) || ~ishghandle(h)
    return
end
typ = object_type(h);
parent = safe_get(h, 'Parent');
ptyp = '';
if ~isempty(parent) && ishghandle(parent)
    ptyp = object_type(parent);
end
if strcmp(typ, 'axes')
    rgb = tick_scene(h);
    return
end
if strcmp(typ, 'text') && strcmp(ptyp, 'axes') && is_axes_label(h, parent)
    rgb = container_background(parent);
    return
end
if any(strcmp(typ, {'text', 'textbox', 'textboxshape'})) && strcmp(ptyp, 'axes')
    rgb = plot_background(parent);
    return
end
if any(strcmp(typ, {'text', 'textbox', 'textboxshape'})) && strcmp(ptyp, 'colorbar')
    rgb = container_background(parent);
    return
end
if any(strcmp(typ, {'uicontrol', 'uipanel', 'uitab', 'uibuttongroup', 'uitable'}))
    rgb = own_or_container(h);
    return
end
rgb = container_background(h);
end


function rgb = tick_scene(ax)
% TickDir out (set before this is used) draws the numbers on the parent.
dir = 'out';
try
    dir = lower(strtrim(char(get(ax, 'TickDir'))));
catch
end
if strcmp(dir, 'in')
    rgb = plot_background(ax);
else
    rgb = container_background(ax);
end
end


function rgb = plot_background(ax)
rgb = color_rgb(safe_get(ax, 'Color'));
if isempty(rgb)
    rgb = container_background(ax);
end
end


function rgb = own_or_container(h)
rgb = color_rgb(safe_get(h, 'BackgroundColor'));
if isempty(rgb)
    rgb = color_rgb(safe_get(h, 'Color'));
end
if isempty(rgb)
    rgb = container_background(h);
end
end


function rgb = container_background(h)
rgb = [];
if ~isempty(h) && ishghandle(h) && strcmp(object_type(h), 'figure')
    rgb = color_rgb(safe_get(h, 'Color'));
    if ~isempty(rgb)
        return
    end
end
p = safe_get(h, 'Parent');
for guard = 1:30
    if isempty(p) || ~ishghandle(p)
        break
    end
    rgb = face_of(p);
    if ~isempty(rgb)
        return
    end
    typ = object_type(p);
    if strcmp(typ, 'figure') || strcmp(typ, 'root')
        break
    end
    p = safe_get(p, 'Parent');
end
rgb = [1 1 1];
end


function rgb = face_of(p)
typ = object_type(p);
switch typ
    case 'figure'
        rgb = color_rgb(safe_get(p, 'Color'));
    case {'uipanel', 'uitab', 'uibuttongroup', 'uicontainer'}
        rgb = color_rgb(safe_get(p, 'BackgroundColor'));
    otherwise
        rgb = [];
end
end


function rgb = legend_box(h)
rgb = [];
cur = h;
for guard = 1:30
    if isempty(cur) || ~ishghandle(cur)
        break
    end
    if strcmp(object_type(cur), 'legend')
        rgb = color_rgb(safe_get(cur, 'Color'));
        if isempty(rgb)
            rgb = color_rgb(safe_get(cur, 'BackgroundColor'));
        end
        break
    end
    cur = safe_get(cur, 'Parent');
end
if isempty(rgb)
    rgb = [1 1 1];
end
end


function tf = is_axes_label(h, ax)
tf = false;
names = {'Title', 'XLabel', 'YLabel', 'ZLabel'};
for n = 1:numel(names)
    label = safe_get(ax, names{n});
    if same_handle(h, label)
        tf = true;
        return
    end
end
end


function ink = choose_ink(bg)
[ink, ~] = choose_ink_plate(bg);
end


function [ink, plate] = choose_ink_plate(bg)
T = theme_struct();
plate = [];
if isempty(bg)
    ink = T.darkText;
    return
end
onLight = contrast_ratio(T.lightText, bg);
onDark = contrast_ratio(T.darkText, bg);
if onLight >= onDark
    ink = T.lightText;
    best = onLight;
else
    ink = T.darkText;
    best = onDark;
end
if best + 1e-9 < T.minContrast
    plate = T.darkText;
    ink = T.lightText;
end
end


function ratio = contrast_ratio(a, b)
ratio = 1;
a = color_rgb(a);
b = color_rgb(b);
if isempty(a) || isempty(b)
    return
end
hi = max(rel_luminance(a), rel_luminance(b));
lo = min(rel_luminance(a), rel_luminance(b));
ratio = (hi + 0.05) / (lo + 0.05);
end


function L = rel_luminance(rgb)
lin = zeros(1, 3);
for k = 1:3
    c = rgb(k);
    if c <= 0.04045
        lin(k) = c / 12.92;
    else
        lin(k) = ((c + 0.055) / 1.055) ^ 2.4;
    end
end
L = 0.2126 * lin(1) + 0.7152 * lin(2) + 0.0722 * lin(3);
end


function rgb = color_rgb(value)
rgb = [];
if isempty(value)
    return
end
if ischar(value) || is_string(value)
    switch lower(strtrim(char(value)))
        case {'w', 'white'}
            rgb = [1 1 1];
        case {'k', 'black'}
            rgb = [0 0 0];
        otherwise
            rgb = [];
    end
    return
end
if isnumeric(value) && numel(value) >= 3
    rgb = double(value(1:3));
    rgb = rgb(:).';
end
end


function tf = is_string(value)
tf = false;
try
    tf = isstring(value);
catch
    tf = false;
end
end


function tf = is_our_plate(h)
tf = false;
try
    tf = isappdata(h, 'ClosedLoopPlate') && logical(getappdata(h, 'ClosedLoopPlate'));
catch
    tf = false;
end
end


function set_plate_flag(h, flag)
try
    setappdata(h, 'ClosedLoopPlate', logical(flag));
catch
end
end


function tf = same_handle(a, b)
tf = false;
if isempty(a) || isempty(b)
    return
end
try
    tf = isequal(a, b);
catch
    tf = false;
end
end


function saved = snapshot_legends(fig)
saved = {};
try
    legs = findall(fig, 'Type', 'legend');
catch
    return
end
names = {'TextColor', 'Color', 'EdgeColor', 'FontSize', 'FontName', ...
    'FontWeight', 'FontAngle', 'FontUnits'};
for i = 1:numel(legs)
    leg = legs(i);
    if ~ishghandle(leg)
        continue
    end
    props = struct();
    for n = 1:numel(names)
        try
            if has_prop(leg, names{n})
                props.(names{n}) = get(leg, names{n});
            end
        catch
        end
    end
    item.handle = leg;
    item.props = props;
    saved{end + 1} = item; %#ok<AGROW>
end
end


function restore_legends(saved)
for i = 1:numel(saved)
    item = saved{i};
    leg = item.handle;
    if ~ishghandle(leg)
        continue
    end
    names = fieldnames(item.props);
    for n = 1:numel(names)
        try_set(leg, names{n}, item.props.(names{n}));
    end
end
end


function tf = is_under_legend(h)
tf = false;
cur = h;
for guard = 1:30
    if isempty(cur) || ~ishghandle(cur)
        return
    end
    typ = object_type(cur);
    if strcmp(typ, 'legend')
        tf = true;
        return
    end
    if strcmp(typ, 'figure') || strcmp(typ, 'root')
        return
    end
    cur = safe_get(cur, 'Parent');
end
end


function typ = object_type(h)
typ = '';
try
    typ = lower(strtrim(char(get(h, 'Type'))));
catch
    typ = '';
end
end


function value = safe_get(h, name)
value = [];
if isempty(h) || ~ishghandle(h)
    return
end
try
    value = get(h, name);
catch
    value = [];
end
end


function tf = has_prop(h, name)
tf = false;
if isempty(h) || ~ishghandle(h)
    return
end
try
    tf = isprop(h, name);
catch
    try
        get(h, name);
        tf = true;
    catch
        tf = false;
    end
end
end


function ok = try_set(h, varargin)
ok = false;
if isempty(h) || ~ishghandle(h)
    return
end
try
    set(h, varargin{:});
    ok = true;
catch
end
end

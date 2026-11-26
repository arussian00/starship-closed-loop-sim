function varargout = closed_loop_theme(action, varargin)
%CLOSED_LOOP_THEME  Colors for the mission-control window.
%
%   T = closed_loop_theme
%   closed_loop_theme('figure', fig)     dark figure, near-white default text
%   closed_loop_theme('graph', ax)       dark plot area, near-white labels
%   closed_loop_theme('overlay', ax)     leave the plot color, near-white labels
%   closed_loop_theme('refresh', fig)    walk text after a redraw
%   closed_loop_theme('reinforce', fig)  same walk, used after a resize
%
%   Graph interiors stay the dark grey in T.plot. Line colors are not set
%   here. Titles, axis labels, tick numbers, panel and tab titles, and
%   text or annotation overlays are #F0F0F0. They sit on the figure as well
%   as inside the axes, so the figure margin is dark too. Gridlines stay
%   the same grey. Legends are left exactly as the caller styled them
%   (white box, near-black type, near-black edge, same font).
%
%   R2025a figure Theme 'light' is applied first so a dark MATLAB Online
%   desktop does not repaint line colors. Explicit colors are set after
%   that, and again on every refresh.

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
    otherwise
        error('closed_loop_theme:Action', 'Unknown action "%s".', char(action));
end
end


function T = theme_struct()
T.figure = [0.10 0.10 0.12];
T.plot = [0.16 0.16 0.18];
T.text = [240 240 240] / 255;
T.inPlot = T.text;
T.grid = [0.62 0.63 0.66];
T.gridAlpha = 0.4;
T.bannerBack = [0.28 0.11 0.10];
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
try_set(fig, 'DefaultAxesXColor', T.text);
try_set(fig, 'DefaultAxesYColor', T.text);
try_set(fig, 'DefaultAxesZColor', T.text);
try_set(fig, 'DefaultTextColor', T.text);
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
        % the dark plot underneath. Its tick labels still turn near-white.
        paint_chrome(partner, T, true);
    end
end
paint_chrome(ax, T, true);
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


function paint_chrome(ax, T, setFont)
try_set(ax, 'XColor', T.text);
try_set(ax, 'YColor', T.text);
try_set(ax, 'ZColor', T.text);
if setFont
    try_set(ax, 'FontSize', T.fontAxes);
end
paint_rulers(ax, T);
paint_label(ax, 'Title', T, setFont);
paint_label(ax, 'XLabel', T, setFont);
paint_label(ax, 'YLabel', T, setFont);
paint_label(ax, 'ZLabel', T, setFont);
end


function paint_rulers(ax, T)
names = {'XAxis', 'YAxis', 'ZAxis'};
for n = 1:numel(names)
    try
        rulers = get(ax, names{n});
    catch
        continue
    end
    for k = 1:numel(rulers)
        try_set(rulers(k), 'Color', T.text);
    end
end
end


function paint_label(ax, which, T, setFont)
try
    h = get(ax, which);
catch
    return
end
paint_text_handle(h, T, setFont, strcmp(which, 'Title'));
end


function paint_text_handle(h, T, setFont, isTitle)
if isempty(h) || ~ishghandle(h) || is_under_legend(h)
    return
end
try_set(h, 'Color', T.text);
try_set(h, 'FontWeight', 'bold');
if ~setFont
    return
end
if isTitle
    try_set(h, 'FontSize', T.fontTitle);
else
    try_set(h, 'FontSize', T.fontAxes);
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
    paint_object(objs(i), T);
end
restore_legends(savedLegends);
end


function paint_object(h, T)
if isempty(h) || ~ishghandle(h) || is_under_legend(h)
    return
end
try
    typ = lower(char(get(h, 'Type')));
catch
    return
end
switch typ
    case 'axes'
        dark = false;
        try
            dark = isappdata(h, 'ClosedLoopGraph') && getappdata(h, 'ClosedLoopGraph');
        catch
            dark = false;
        end
        if dark
            paint_plot(h, T);
        end
        paint_chrome(h, T, false);
    case {'text', 'textbox', 'textboxshape'}
        try_set(h, 'Color', T.text);
    case 'hggroup'
        % Octave annotation textboxes are hggroup objects. Color is the
        % text. Groups without a String are series containers, left alone.
        if has_prop(h, 'String')
            try_set(h, 'Color', T.text);
        end
    case {'textarrow', 'textarrowshape'}
        try_set_if_prop(h, 'TextColor', T.text);
        try_set_if_prop(h, 'FontColor', T.text);
    case 'colorbar'
        try_set(h, 'Color', T.text);
        try
            paint_text_handle(get(h, 'Label'), T, false, false);
        catch
        end
        try
            paint_text_handle(get(h, 'Title'), T, false, true);
        catch
        end
    case {'uicontrol', 'uipanel', 'uitab', 'uibuttongroup', 'uitable'}
        try_set(h, 'ForegroundColor', T.text);
        try_set_if_prop(h, 'FontColor', T.text);
    otherwise
        try_set_if_prop(h, 'FontColor', T.text);
        try_set_if_prop(h, 'TextColor', T.text);
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
            if isprop(leg, names{n})
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
    try
        typ = lower(char(get(cur, 'Type')));
    catch
        return
    end
    if strcmp(typ, 'legend')
        tf = true;
        return
    end
    if strcmp(typ, 'figure') || strcmp(typ, 'root')
        return
    end
    try
        cur = get(cur, 'Parent');
    catch
        return
    end
end
end


function try_set_if_prop(h, name, value)
if has_prop(h, name)
    try_set(h, name, value);
end
end


function tf = has_prop(h, name)
tf = false;
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
try
    set(h, varargin{:});
    ok = true;
catch
end
end

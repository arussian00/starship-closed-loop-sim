function check_figure_text()
%CHECK_FIGURE_TEXT  Every text object clears WCAG contrast 4.5.
%
%   check_figure_text
%
%   Builds an invisible figure with a dark graph, light and dark camera
%   skies, the mid-sky tone that neither ink can clear, light and dark
%   panels and controls, and a legend. After closed_loop_theme refreshes
%   the figure, each text color is checked against the background it
%   actually uses. Legends and line colors stay as they were set.

here = fileparts(mfilename('fullpath'));
if isempty(here)
    here = pwd;
end
addpath(here);
try
    graphics_toolkit('gnuplot');
catch
end

T = closed_loop_theme();
assert_color(T.figure, [1 1 1], 'theme figure');
assert_color(T.plot, [0.16 0.16 0.18], 'theme plot');
assert_color(T.grid, [0.62 0.63 0.66], 'theme grid');
assert_color(T.lightText, [240 240 240] / 255, 'theme light text');
assert_color(T.darkText, [0.1 0.1 0.1], 'theme dark text');
assert_color(T.inPlot, T.lightText, 'theme inPlot');
assert_color(T.text, T.darkText, 'theme text');
assert_color(T.bannerBack, [1 0.96 0.93], 'theme banner');
assert_color(T.legendText, [0.1 0.1 0.1], 'theme legend text');
assert_color(T.legendBack, [1 1 1], 'theme legend back');
assert_color(T.legendEdge, [0.1 0.1 0.1], 'theme legend edge');
if abs(T.minContrast - 4.5) > 1e-12
    error('check_figure_text:Contrast', 'Minimum contrast is %g.', T.minContrast);
end

fig = figure('Visible', 'off', 'Color', [1 1 1]);
try
    exercise(fig, T);
catch ME
    close_quietly(fig);
    rethrow(ME);
end
close_quietly(fig);
fprintf('check_figure_text: contrast >= 4.5, legend unchanged, lines unchanged.\n');
end


function exercise(fig, T)
lightSky = [0.53 0.81 0.98];
darkSky = [0.02 0.02 0.08];
midSky = [0.3209 0.4861 0.611];
lineColor = [0.85 0.25 0.15];
lightFace = [0.96 0.96 0.96];
darkFace = [0.12 0.12 0.14];

closed_loop_theme('figure', fig);

ax = axes('Parent', fig, 'Position', [0.08 0.55 0.35 0.3]);
hold(ax, 'on');
grid(ax, 'on');
hLine = plot(ax, 1:4, [1 3 2 4], '-', 'Color', lineColor, 'LineWidth', 1.5);
hBlack = plot(ax, 1:4, [0 1 0 1], 'k--', 'LineWidth', 1);
hMark = plot(ax, 2, 2, 'k^', 'MarkerFaceColor', [1 0.8 0], 'MarkerSize', 7);
title(ax, 'Pitch [deg]');
xlabel(ax, 'Time [min]');
ylabel(ax, 'Altitude [km]');
hRead = text(ax, 0.02, 0.95, 'after cutoff: guidance (controller off)', ...
    'Units', 'normalized', 'Color', [0 0 0], 'VerticalAlignment', 'top');
hLive = text(ax, 0.98, 0.97, 'T+00:00', 'Units', 'normalized', ...
    'Color', [0 0 0], 'HorizontalAlignment', 'right');
leg = legend(ax, [hLine, hBlack], {'Super Heavy', 'nominal'}, ...
    'Location', 'northwest');
style_like_animation(leg, T);
set(leg, 'FontSize', 12);
closed_loop_theme('graph', ax);

axLight = axes('Parent', fig, 'Position', [0.55 0.55 0.35 0.3], 'Color', lightSky);
title(axLight, 'Light sky');
hLight = text(axLight, 0.1, 0.8, 'HUD light', 'Units', 'normalized', 'Color', [1 1 1]);
closed_loop_theme('overlay', axLight);

axDark = axes('Parent', fig, 'Position', [0.08 0.15 0.35 0.3], 'Color', darkSky);
title(axDark, 'Dark sky');
hDark = text(axDark, 0.1, 0.8, 'HUD dark', 'Units', 'normalized', 'Color', [0 0 0]);
closed_loop_theme('overlay', axDark);

axMid = axes('Parent', fig, 'Position', [0.55 0.15 0.35 0.3], 'Color', midSky);
title(axMid, 'Mid sky');
hMid = text(axMid, 0.1, 0.8, 'HUD mid', 'Units', 'normalized', 'Color', [0.5 0.5 0.5]);
closed_loop_theme('overlay', axMid);

hAnnLight = annotation(fig, 'textbox', [0.08 0.90 0.35 0.05], ...
    'String', 'CLOSED LOOP', 'Color', [1 1 1], ...
    'BackgroundColor', T.bannerBack, 'EdgeColor', 'none');
hAnnDark = annotation(fig, 'textbox', [0.55 0.90 0.35 0.05], ...
    'String', 'controller off', 'Color', [0 0 0], ...
    'BackgroundColor', darkFace, 'EdgeColor', 'none');

hPanelLight = uipanel('Parent', fig, 'Title', 'Light panel', ...
    'Units', 'pixels', 'Position', [4 4 90 28], ...
    'BackgroundColor', lightFace, 'ForegroundColor', [1 1 1]);
hPanelDark = uipanel('Parent', fig, 'Title', 'Dark panel', ...
    'Units', 'pixels', 'Position', [100 4 90 28], ...
    'BackgroundColor', darkFace, 'ForegroundColor', [0 0 0]);
hUiLight = uicontrol('Parent', fig, 'Style', 'text', 'String', 'Light', ...
    'Units', 'pixels', 'Position', [4 36 70 16], ...
    'BackgroundColor', lightFace, 'ForegroundColor', [1 1 1]);
hUiDark = uicontrol('Parent', fig, 'Style', 'text', 'String', 'Dark', ...
    'Units', 'pixels', 'Position', [100 36 70 16], ...
    'BackgroundColor', darkFace, 'ForegroundColor', [0 0 0]);
hTab = [];
if exist('uitabgroup', 'file') == 2 || exist('uitabgroup', 'builtin') == 5
    grp = uitabgroup(fig, 'Units', 'pixels', 'Position', [200 4 80 40]);
    hTab = uitab(grp, 'Title', 'Tab', 'BackgroundColor', lightFace);
    try_set(hTab, 'ForegroundColor', [1 1 1]);
end

closed_loop_theme('refresh', fig);

assert_color(get(fig, 'Color'), T.figure, 'figure');
assert_color(get(ax, 'Color'), T.plot, 'graph interior');
assert_color(get(ax, 'GridColor'), T.grid, 'grid');
assert_color(get(ax, 'XColor'), T.darkText, 'tick on white figure');
assert_color(get(get(ax, 'Title'), 'Color'), T.darkText, 'title on white figure');
assert_color(get(get(ax, 'XLabel'), 'Color'), T.darkText, 'xlabel on white figure');
assert_color(get(get(ax, 'YLabel'), 'Color'), T.darkText, 'ylabel on white figure');
assert_color(get(hRead, 'Color'), T.lightText, 'controller-off on dark plot');
assert_color(get(hLive, 'Color'), T.lightText, 'live readout on dark plot');
assert_no_plate(hRead);
assert_color(get(hLine, 'Color'), lineColor, 'booster line');
assert_color(get(hBlack, 'Color'), [0 0 0], 'black line');
assert_color(get(hMark, 'Color'), [0 0 0], 'pad marker');
assert_color(get(hMark, 'MarkerFaceColor'), [1 0.8 0], 'pad marker face');
assert_color(get(axLight, 'Color'), lightSky, 'light sky scene');
assert_color(get(hLight, 'Color'), T.darkText, 'text on light sky');
assert_no_plate(hLight);
assert_color(get(get(axLight, 'Title'), 'Color'), T.darkText, 'title above light sky');
assert_color(get(axDark, 'Color'), darkSky, 'dark sky scene');
assert_color(get(hDark, 'Color'), T.lightText, 'text on dark sky');
assert_no_plate(hDark);
assert_color(get(axMid, 'Color'), midSky, 'mid sky scene');
assert_color(get(hMid, 'Color'), T.lightText, 'text on mid sky');
assert_color(get(hMid, 'BackgroundColor'), T.darkText, 'mid sky plate');
assert_color(get(hAnnLight, 'Color'), T.darkText, 'banner text');
assert_color(get(hAnnLight, 'BackgroundColor'), T.bannerBack, 'banner background');
assert_color(get(hAnnDark, 'Color'), T.lightText, 'dark annotation text');
assert_color(get(hAnnDark, 'BackgroundColor'), darkFace, 'dark annotation background');
assert_color(get(hPanelLight, 'BackgroundColor'), lightFace, 'light panel face');
assert_prop_color(hPanelLight, 'ForegroundColor', T.darkText, 'light panel title');
assert_color(get(hPanelDark, 'BackgroundColor'), darkFace, 'dark panel face');
assert_prop_color(hPanelDark, 'ForegroundColor', T.lightText, 'dark panel title');
assert_prop_color(hUiLight, 'ForegroundColor', T.darkText, 'light control');
assert_prop_color(hUiDark, 'ForegroundColor', T.lightText, 'dark control');
if ~isempty(hTab) && ishghandle(hTab)
    assert_prop_color(hTab, 'ForegroundColor', T.darkText, 'light tab');
end
assert_legend_held(leg, T);
audit_contrast(fig, T);

set(hLive, 'String', 'T+01:02', 'Color', [0 0 0]);
set(hLight, 'Color', T.lightText);
set(axDark, 'Color', lightSky);
set(hDark, 'Color', T.lightText);
set(axMid, 'Color', darkSky);
closed_loop_theme('refresh', fig);
assert_color(get(hLive, 'Color'), T.lightText, 'live readout after update');
assert_color(get(hLight, 'Color'), T.darkText, 'light sky text after update');
assert_color(get(axDark, 'Color'), lightSky, 'sky recolored');
assert_color(get(hDark, 'Color'), T.darkText, 'text follows the new sky');
assert_no_plate(hDark);
assert_color(get(axMid, 'Color'), darkSky, 'mid axes moved to dark sky');
assert_color(get(hMid, 'Color'), T.lightText, 'text on the darkened sky');
assert_no_plate(hMid);
assert_color(get(hLine, 'Color'), lineColor, 'booster line after update');
assert_legend_held(leg, T);
audit_contrast(fig, T);

hSg = [];
try
    hSg = sgtitle(fig, 'Summary');
catch
    hSg = [];
end
if ~isempty(hSg) && ishghandle(hSg)
    set(hSg, 'Color', T.lightText);
    closed_loop_theme('refresh', fig);
    assert_color(get(hSg, 'Color'), T.darkText, 'sgtitle on white figure');
    audit_contrast(fig, T);
    assert_legend_held(leg, T);
end
end


function style_like_animation(leg, T)
try_set(leg, 'TextColor', T.legendText);
try_set(leg, 'Color', T.legendBack);
try_set(leg, 'EdgeColor', T.legendEdge);
try_set(leg, 'FontSize', T.fontSmall);
end


function audit_contrast(fig, T)
n = 0;
try
    objs = findall(fig);
catch
    objs = [];
end
for i = 1:numel(objs)
    h = objs(i);
    if isempty(h) || ~ishghandle(h)
        continue
    end
    typ = '';
    try
        typ = lower(strtrim(char(get(h, 'Type'))));
    catch
        continue
    end
    switch typ
        case 'axes'
            n = n + check_one(h, 'XColor', T, 'XColor');
            n = n + check_one(h, 'YColor', T, 'YColor');
            n = n + check_one(h, 'ZColor', T, 'ZColor');
        case {'text', 'textbox', 'textboxshape'}
            n = n + check_one(h, 'Color', T, 'text');
        case 'hggroup'
            if has_prop(h, 'String')
                n = n + check_one(h, 'Color', T, 'annotation');
            end
        case 'legend'
            n = n + check_one(h, 'TextColor', T, 'legend');
        case 'colorbar'
            n = n + check_one(h, 'Color', T, 'colorbar');
        case {'uicontrol', 'uipanel', 'uitab', 'uibuttongroup', 'uitable'}
            n = n + check_one(h, 'ForegroundColor', T, typ);
    end
end
if n < 8
    error('check_figure_text:Contrast', 'Only %d text colors were checked.', n);
end
end


function n = check_one(h, prop, T, label)
n = 0;
if ~has_prop(h, prop)
    return
end
try
    ink = get(h, prop);
catch
    return
end
if ~(isnumeric(ink) && numel(ink) >= 3)
    return
end
bg = closed_loop_theme('background', h);
ratio = closed_loop_theme('ratio', ink, bg);
if ~(isnumeric(ratio) && ratio + 1e-6 >= T.minContrast)
    error('check_figure_text:Contrast', ...
        '%s contrast %.3f is below %.1f (ink [%s], background [%s]).', ...
        label, double(ratio), T.minContrast, vec_text(ink), vec_text(bg));
end
n = 1;
end


function assert_legend_held(leg, T)
assert_prop_color(leg, 'TextColor', T.legendText, 'legend TextColor');
assert_color(get(leg, 'Color'), T.legendBack, 'legend Color');
assert_color(get(leg, 'EdgeColor'), T.legendEdge, 'legend EdgeColor');
got = get(leg, 'FontSize');
if abs(double(got) - 12) > 1e-9
    error('check_figure_text:Font', 'Legend FontSize changed to %g.', double(got));
end
end


function assert_no_plate(h)
if ~has_prop(h, 'BackgroundColor')
    return
end
value = get(h, 'BackgroundColor');
if ischar(value) && strcmpi(strtrim(value), 'none')
    return
end
error('check_figure_text:Plate', 'Text gained a plate on a background that already contrasts.');
end


function assert_prop_color(h, name, expected, label)
if ~has_prop(h, name)
    error('check_figure_text:Color', '%s has no %s.', label, name);
end
assert_color(get(h, name), expected, label);
end


function assert_color(actual, expected, label)
if ischar(actual)
    error('check_figure_text:Color', '%s is "%s", expected RGB.', label, actual);
end
try
    actual = double(actual(:));
    expected = double(expected(:));
catch
    error('check_figure_text:Color', '%s is not numeric RGB.', label);
end
if numel(actual) ~= 3 || max(abs(actual - expected)) > 1.5 / 255
    error('check_figure_text:Color', '%s is [%s], expected [%s].', ...
        label, vec_text(actual), vec_text(expected));
end
end


function text = vec_text(value)
if isempty(value) || ~isnumeric(value)
    text = '?';
    return
end
text = sprintf('%.5f ', double(value(:)));
text = strtrim(text);
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


function try_set(h, name, value)
try
    set(h, name, value);
catch
end
end


function close_quietly(fig)
try
    if ishghandle(fig)
        close(fig);
    end
catch
end
end

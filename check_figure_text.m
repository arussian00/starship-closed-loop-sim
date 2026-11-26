function check_figure_text()
%CHECK_FIGURE_TEXT  Titles, ticks, and overlays are #F0F0F0. Legends are not.
%
%   check_figure_text
%
%   Builds an invisible figure, paints it with closed_loop_theme, then
%   checks text, a legend, line colors, and the dark graph interior.
%   Needs a MATLAB or Octave graphics session. Visible stays off.

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
assert_color(T.text, [240 240 240] / 255, 'theme text');
assert_color(T.inPlot, T.text, 'theme inPlot');
assert_color(T.plot, [0.16 0.16 0.18], 'theme plot');
assert_color(T.grid, [0.62 0.63 0.66], 'theme grid');
assert_color(T.legendText, [0.1 0.1 0.1], 'theme legend text');
assert_color(T.legendBack, [1 1 1], 'theme legend back');
assert_color(T.legendEdge, [0.1 0.1 0.1], 'theme legend edge');
if max(T.figure) >= 0.25
    error('check_figure_text:Figure', 'Figure color is not dark.');
end

fig = figure('Visible', 'off', 'Color', [1 1 1]);
try
    exercise(fig, T);
catch ME
    close_quietly(fig);
    rethrow(ME);
end
close_quietly(fig);
fprintf('check_figure_text: near-white text, legend unchanged, lines unchanged.\n');
end


function exercise(fig, T)
closed_loop_theme('figure', fig);
assert_color(get(fig, 'Color'), T.figure, 'figure');

ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
lineColor = [0.85 0.25 0.15];
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
closed_loop_theme('graph', ax);
style_like_animation(leg, T);
set(leg, 'FontSize', 12);

axOver = axes('Parent', fig, 'Position', [0.55 0.15 0.3 0.3]);
hold(axOver, 'on');
title(axOver, 'Earth view');
xlabel(axOver, 'km');
overlayColor = [0.20 0.40 0.60];
set(axOver, 'Color', overlayColor);
closed_loop_theme('overlay', axOver);

hAnn = [];
try
    hAnn = annotation(fig, 'textbox', [0.15 0.88 0.4 0.06], ...
        'String', 'CLOSED LOOP', 'Color', [0 0 0], ...
        'BackgroundColor', [0.28 0.11 0.10], 'EdgeColor', 'none');
catch
    hAnn = [];
end

hPanel = [];
try
    hPanel = uipanel('Parent', fig, 'Title', 'Panel', ...
        'Units', 'pixels', 'Position', [4 4 40 24], ...
        'ForegroundColor', [0 0 0]);
catch
    hPanel = [];
end

hUi = [];
try
    hUi = uicontrol('Parent', fig, 'Style', 'text', 'String', 'Readout', ...
        'Units', 'pixels', 'Position', [4 32 80 16], ...
        'ForegroundColor', [0 0 0]);
catch
    hUi = [];
end

hTab = [];
try
    grp = uitabgroup(fig);
    hTab = uitab(grp, 'Title', 'Tab');
    try_set(hTab, 'ForegroundColor', [0 0 0]);
catch
    hTab = [];
end

closed_loop_theme('refresh', fig);

assert_color(get(fig, 'Color'), T.figure, 'figure after refresh');
assert_color(get(ax, 'Color'), T.plot, 'graph interior');
assert_color(get(ax, 'GridColor'), T.grid, 'grid');
assert_color(get(ax, 'XColor'), T.text, 'XColor');
assert_color(get(ax, 'YColor'), T.text, 'YColor');
assert_color(get(ax, 'ZColor'), T.text, 'ZColor');
assert_color(get(get(ax, 'Title'), 'Color'), T.text, 'title');
assert_color(get(get(ax, 'XLabel'), 'Color'), T.text, 'xlabel');
assert_color(get(get(ax, 'YLabel'), 'Color'), T.text, 'ylabel');
assert_color(get(hRead, 'Color'), T.text, 'controller-off text');
assert_color(get(hLive, 'Color'), T.text, 'live readout');
assert_color(get(hLine, 'Color'), lineColor, 'booster line');
assert_color(get(hBlack, 'Color'), [0 0 0], 'black line');
assert_color(get(hMark, 'Color'), [0 0 0], 'pad marker');
assert_color(get(hMark, 'MarkerFaceColor'), [1 0.8 0], 'pad marker face');
assert_color(get(axOver, 'Color'), overlayColor, 'overlay interior');
assert_color(get(axOver, 'XColor'), T.text, 'overlay XColor');
assert_color(get(get(axOver, 'Title'), 'Color'), T.text, 'overlay title');
assert_legend_held(leg, T);

if ~isempty(hAnn) && ishghandle(hAnn)
    assert_color(get(hAnn, 'Color'), T.text, 'annotation');
    assert_color(get(hAnn, 'BackgroundColor'), [0.28 0.11 0.10], 'annotation background');
end
if ~isempty(hPanel) && ishghandle(hPanel)
    assert_prop_color(hPanel, 'ForegroundColor', T.text, 'uipanel title');
end
if ~isempty(hUi) && ishghandle(hUi)
    assert_prop_color(hUi, 'ForegroundColor', T.text, 'uicontrol');
end
if ~isempty(hTab) && ishghandle(hTab)
    assert_prop_color(hTab, 'ForegroundColor', T.text, 'uitab title');
end

set(hLive, 'String', 'T+01:02', 'Color', [0 0 0]);
set(get(ax, 'Title'), 'Color', [0 0 0]);
closed_loop_theme('refresh', fig);
assert_color(get(hLive, 'Color'), T.text, 'live readout after update');
assert_color(get(get(ax, 'Title'), 'Color'), T.text, 'title after update');
assert_color(get(hLine, 'Color'), lineColor, 'booster line after update');
assert_color(get(axOver, 'Color'), overlayColor, 'overlay interior after update');
assert_legend_held(leg, T);

hSg = [];
try
    hSg = sgtitle(fig, 'Summary');
catch
    hSg = [];
end
if ~isempty(hSg) && ishghandle(hSg)
    set(hSg, 'Color', [0 0 0]);
    closed_loop_theme('refresh', fig);
    assert_color(get(hSg, 'Color'), T.text, 'sgtitle');
    assert_legend_held(leg, T);
end
end


function style_like_animation(leg, T)
try_set(leg, 'TextColor', T.legendText);
try_set(leg, 'Color', T.legendBack);
try_set(leg, 'EdgeColor', T.legendEdge);
try_set(leg, 'FontSize', T.fontSmall);
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


function assert_prop_color(h, name, expected, label)
try
    if ~isprop(h, name)
        fprintf('check_figure_text: %s has no %s (skipped).\n', label, name);
        return
    end
    actual = get(h, name);
catch ME
    fprintf('check_figure_text: %s skipped (%s).\n', label, ME.message);
    return
end
assert_color(actual, expected, label);
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
if isempty(value)
    text = '';
    return
end
text = sprintf('%.5f ', value(:));
text = strtrim(text);
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

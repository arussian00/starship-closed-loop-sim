function L = mission_control_layout()
%MISSION_CONTROL_LAYOUT  Panel positions shared with the playback window.
%
%   Columns match animateStarship (x = 0.035, 0.285, 0.535, 0.785, width
%   0.20). The bottom row is the closed-loop controls strip.

L.col = [0.035 0.285 0.535 0.785];
L.wid = 0.20;
L.rows.scene = [0.700 0.230];
L.rows.upper = [0.490 0.175];
L.rows.lower = [0.280 0.175];
L.rows.controls = [0.040 0.205];

L.telemNames = { ...
    'Altitude [km]', 'Speed [km/s]', 'g-load [g]', 'Dynamic pressure [kPa]', ...
    'Throttle [%]', 'Angle of attack [deg]', 'Mass [t] (log)', 'Heating [kW/m^2]'};
L.controlNames = { ...
    'Pitch command / actual [deg]', ...
    'Gimbal [deg] and rate [deg/s]', ...
    'AoA [deg] and q*alpha [kPa deg]', ...
    'Tracking error [deg]'};

L.figureName = 'Starship mission control';
L.indicatorClosed = 'CLOSED LOOP (controller flying)';
L.indicatorGuidance = 'guidance (controller off)';

L.color.booster = [0.85 0.25 0.15];
L.color.ship = [0.10 0.35 0.85];
L.color.command = [0.93 0.93 0.93];
L.color.response = [0.35 0.75 0.95];
L.color.error = [0.95 0.55 0.40];
L.color.limit = [0.90 0.45 0.40];
L.color.rate = [0.95 0.75 0.35];
L.color.nominal = [0.78 0.78 0.82];
L.color.banner = [0.55 0.08 0.05];
end

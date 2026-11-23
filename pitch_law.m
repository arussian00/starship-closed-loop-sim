function D = pitch_law(V)
%PITCH_LAW  Cascade gains for the discrete pitch controller.
%
%   D = pitch_law(V)
%
%   Same numbers design_pitch_controller uses when Kp, Ki, and Kd are not
%   overridden: one PID at every condition, with mu and lambda scheduled
%   outside this function. Kq is the inner rate gain, Kth the attitude
%   gain, KiTh the attitude integrator.
%
%       Kp = Kq*Kth,  Kd = Kq,  Ki = Kq*KiTh
%       delta = a_cmd / mu
%
%   When design_pitch_controller runs (Control System Toolbox), its gains
%   replace the formulas so a paste-back in vehicle_params is honored.
%   The toolbox call is skipped when it is not available. D keeps only
%   numeric fields, so the flight loop does not need the toolbox.

D = law_from_vehicle(V);
D.source = 'vehicle_params formulas (design_pitch_controller defaults)';
D.flight = V.flight;

if exist('design_pitch_controller', 'file') ~= 2
    return
end
try
    Dd = design_pitch_controller(V, 'plot', false, 'verbose', false);
catch ME
    D.source = ['vehicle_params formulas; design_pitch_controller was not used: ' ...
        flatten_message(ME.message)];
    return
end
D = copy_gain_fields(D, Dd);
D.source = 'design_pitch_controller';
D.flight = V.flight;
end


function D = law_from_vehicle(V)
D.Kq = V.Kq;
D.Kth = V.attWn^2 / D.Kq;
D.KiTh = V.attKiOverWn3 * V.attWn^3 / D.Kq;
D.Kp = D.Kq * D.Kth;
D.Ki = D.Kq * D.KiTh;
D.Kd = D.Kq;
D.Tf = V.attTf;
D.kaw = V.kaw;
D.cmdTau = V.cmdTau;
D.etaAuth = V.etaAuth;
D.shapeKs = V.shapeKs;
D.trimTau = V.trimTau;
D.shapeQ = D.etaAuth / max(D.Kq, 1e-9);
D.shapeE = D.etaAuth / max(D.Kp, 1e-6);
D.qLimFloor = deg2rad(V.thetaStepDeg) / V.cmdTau * 1.25;
D.eLimFloor = deg2rad(V.thetaStepDeg) * 1.2;
D.deltaAvailFloor = V.deltaAvailFrac * V.gimbalMax;
D.sensorDelay = V.sensorDelay;
D.actHz = V.actHz;
D.actWn = V.actWn;
D.actZeta = V.actZeta;
D.gimbalRate = V.gimbalRate;
D.gimbalRateDeg = V.gimbalRateDeg;
D.deltaMax = V.gimbalMax;
D.deltaMaxDeg = V.gimbalMaxDeg;
D.thetaStepDeg = V.thetaStepDeg;
D.thetaStepTime = V.thetaStepTime;
D.gustSpeed = V.gustSpeed;
D.gustTime = V.gustTime;
D.tStop = V.tStop;
D.vehicleName = V.vehicleName;
D.CNa = V.CNa;
D.diam = V.diam;
D.area = V.area;
D.stackLength = V.stackLength;
D.staticMarginAscent = V.staticMarginAscent;
D.staticMarginLanding = V.staticMarginLanding;
end


function D = copy_gain_fields(D, Dd)
names = {'Kq', 'Kth', 'KiTh', 'Kp', 'Ki', 'Kd', 'Tf', 'kaw', 'cmdTau', ...
    'etaAuth', 'shapeKs', 'trimTau', 'shapeQ', 'shapeE', 'qLimFloor', ...
    'eLimFloor', 'deltaAvailFloor', 'sensorDelay', 'actWn', 'actZeta', ...
    'gimbalRate', 'deltaMax', 'thetaStepDeg', 'thetaStepTime', ...
    'gustSpeed', 'gustTime', 'tStop'};
for k = 1:numel(names)
    name = names{k};
    if isfield(Dd, name) && isnumeric(Dd.(name)) && isscalar(Dd.(name))
        D.(name) = Dd.(name);
    end
end
if isfield(Dd, 'kaw') && isnumeric(Dd.kaw) && isscalar(Dd.kaw)
    D.kaw = Dd.kaw;
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

function flight = nominal_flight()
%NOMINAL_FLIGHT  Open-loop Starship mission from matlab-rocket-sim.
%
%   flight = nominal_flight()
%
%   Ideal steering on the ascent (starshipStack), then simulateBooster for
%   the booster return and the ship's belly-flop. No figures. This is the
%   overlay and the catch/splashdown baseline. The source files are not
%   modified.

[S, Pb, Ps, gravMu, m0] = mission_params();
fprintf('Liftoff mass %.0f t, thrust %.1f MN, thrust-to-weight %.2f\n', ...
    m0 / 1e3, S.T1 / 1e6, S.T1 / (m0 * Pb.g0));

Y0 = [0; 0; 0; 0; m0];
mMECO = m0 - (S.m1prop - S.reserve1);
mShip = S.m2dry + S.m2prop + S.mPay;
opts = odeset('Events', @(t, Y) evMECO(t, Y, mMECO), ...
    'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tA, YA] = ode45(@(t, Y) starshipStack(t, Y, S, Pb, gravMu, 'stack'), [0 400], Y0, opts);
tSep = tA(end);
Ysep = YA(end, :).';
fprintf(['Nominal hot staging at t = %.1f s, h = %.1f km, speed = %.0f m/s, ' ...
    'flight-path angle = %.1f deg\n'], ...
    tSep, Ysep(2) / 1e3, hypot(Ysep(3), Ysep(4)), atan2d(Ysep(4), Ysep(3)));

mBoost = Ysep(5) - mShip;
opts = odeset('Events', @(t, Y) perigeeSECO(t, Y, S, Pb, gravMu), ...
    'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tC, YC] = ode45(@(t, Y) starshipStack(t, Y, S, Pb, gravMu, 'ship'), ...
    [tSep, tSep + 1500], [Ysep(1:4); mShip], opts);
tSECO = tC(end);
Yseco = YC(end, :).';
[perigee, apogee] = orbitOf(Yseco, Pb.Re, gravMu);

Xb0 = [Ysep(1:4); mBoost; atan2(Ysep(3), Ysep(4)); Ysep(3) / (Pb.Re + Ysep(2))];
outB = simulateBooster(Pb, Xb0, tSep, Pb.phaseList);

PsShip = Ps;
PsShip.m_dry = S.m2dry + S.mPay;
Xs0 = [Yseco(1:4); Yseco(5); atan2(Yseco(3), Yseco(4)); Yseco(3) / (Pb.Re + Yseco(2))];
outS = simulateBooster(PsShip, Xs0, tSECO, PsShip.phaseList);

segA = pointMassSegment(tA, YA, Pb, @(t, Y) starshipStack(t, Y, S, Pb, gravMu, 'stack'), S.T1);
segC = pointMassSegment(tC, YC, Pb, @(t, Y) starshipStack(t, Y, S, Pb, gravMu, 'ship'), S.T2);
fullB = fullFlight({segA}, {'stack ascent'}, outB);
fullS = fullFlight({segA, segC}, {'stack ascent', 'ship burn'}, outS);

flight.S = S;
flight.Pb = Pb;
flight.Ps = PsShip;
flight.gravMu = gravMu;
flight.m0 = m0;
flight.mShip = mShip;
flight.mMECO = mMECO;
flight.tSep = tSep;
flight.tSECO = tSECO;
flight.Ysep = Ysep;
flight.Yseco = Yseco;
flight.perigee = perigee;
flight.apogee = apogee;
flight.tA = tA;
flight.YA = YA;
flight.tC = tC;
flight.YC = YC;
flight.outB = outB;
flight.outS = outS;
flight.fullB = fullB;
flight.fullS = fullS;
flight.metricsB = flightMetrics(fullB, false);
flight.metricsS = flightMetrics(fullS, false);
fprintf('Nominal catch miss %.2f m, splashdown miss %.2f m\n', ...
    flight.metricsB.miss, flight.metricsS.miss);
end

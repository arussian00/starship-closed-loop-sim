function [S, Pb, Ps, gravMu, m0] = mission_params()
%MISSION_PARAMS  The Starship flight numbers from rocket_starship_sim.
%
%   Same values run_starship_flight and rocket_starship_sim use, so the
%   open-loop reference and the closed-loop ascent share one vehicle.

Pb = rocket_params('superheavy');
Ps = rocket_params('starship');
gravMu = Pb.g0 * Pb.Re^2;

S.m1dry = Pb.m_dry;
S.m1prop = Pb.m_prop;
S.T1 = Pb.engAscent * Pb.T_eng;
S.Isp1sl = Pb.Isp_sl;
S.Isp1vac = Pb.Isp_vac;
S.reserve1 = 400e3;

S.m2dry = Ps.m_dry;
S.m2prop = Ps.m_prop;
S.T2 = 3 * 2.3e6 + 3 * 2.5e6;
S.Isp2 = 365;
S.reserve2 = 30e3;
S.mPay = 10e3;
S.gLimit = 4;

S.Cd = 0.4;
S.thrBucket = 0.75;
S.bucketMach = [0.6 0.8 1.5 1.8];
S.A = Pb.A;

S.tKick = 8;
S.kickDur = 5;
S.kickDeg = 0.8;

S.hCoast = 190e3;
S.perigee = -50e3;
S.wnG = 0.01;
S.zG = 0.8;

m0 = S.m1dry + S.m1prop + S.m2dry + S.m2prop + S.mPay;
end

function st = discrete_pitch_init(D, dt)
%DISCRETE_PITCH_INIT  State for one fixed-rate copy of boosterPitchTVC.
%
%   The Simulink controller is a command prefilter, a lead-and-rate
%   governor, an attitude loop, an inner rate loop, back-calculation
%   anti-windup, and delta_cmd = a_cmd/mu. This samples that law at dt.
%   Linear filters use the exact one-step map. The governor, the attitude
%   integrator, and the saturations use forward Euler. The sensor delay is
%   an integer number of steps (2 steps at 100 Hz for the 20 ms delay).

st.dt = dt;
st.Kq = D.Kq;
st.Kth = D.Kth;
st.KiTh = D.KiTh;
st.Tf = D.Tf;
st.kaw = D.kaw;
st.cmdTau = D.cmdTau;
st.shapeKs = D.shapeKs;
st.trimTau = D.trimTau;
st.shapeQ = D.shapeQ;
st.shapeE = D.shapeE;
st.qLimFloor = D.qLimFloor;
st.eLimFloor = D.eLimFloor;
st.deltaAvailFloor = D.deltaAvailFloor;
st.deltaMax = D.deltaMax;
st.shapeOn = true;
st.nDelay = max(0, round(D.sensorDelay / max(dt, eps)));
st.delayMiss = abs(st.nDelay * dt - D.sensorDelay);
if st.nDelay == 0
    st.thetaBuf = 0;
    st.qBuf = 0;
else
    st.thetaBuf = zeros(st.nDelay, 1);
    st.qBuf = zeros(st.nDelay, 1);
end
st.pref = 0;
st.qf = 0;
st.trim = 0;
st.shaped = 0;
st.integ = 0;
st.deltaCmd = 0;
end

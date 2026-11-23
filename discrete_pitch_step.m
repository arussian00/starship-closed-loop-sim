function [st, deltaCmd, info] = discrete_pitch_step(st, theta, q, delta, thetaCmd, mu)
%DISCRETE_PITCH_STEP  One control period of the boosterPitchTVC law.
%
%   theta and q are pitch from the trim attitude and its rate. mu is
%   F*arm/I right now. The returned gimbal command is held until the next
%   call. See build_pitch_model for the continuous diagram this samples.

dt = st.dt;
[thetaD, st.thetaBuf] = delay_push(st.thetaBuf, theta, st.nDelay);
[qD, st.qBuf] = delay_push(st.qBuf, q, st.nDelay);
st.pref = lpf_step(st.pref, thetaCmd, st.cmdTau, dt);
st.qf = lpf_step(st.qf, qD, st.Tf, dt);
st.trim = lpf_step(st.trim, delta, st.trimTau, dt);

info.thetaDelayed = thetaD;
info.qDelayed = qD;
info.pref = st.pref;
info.e = thetaCmd - thetaD;
info.qCmd = 0;
info.aCmd = 0;
info.saturated = false;

if ~(isfinite(mu) && mu > 1e-4)
    st.deltaCmd = 0;
    deltaCmd = 0;
    info.e = 0;
    return
end

avail = max(st.deltaMax - abs(st.trim), st.deltaAvailFloor);
aAvail = mu * sin(cl_clip(avail, 0, pi / 2));
qLim = max(st.shapeQ * aAvail, st.qLimFloor);
eLim = max(st.shapeE * aAvail, st.eLimFloor);

lead = cl_clip(st.pref - thetaD, -eLim, eLim);
target = thetaD + lead;
sdot = cl_clip(st.shapeKs * (target - st.shaped), -qLim, qLim);
st.shaped = st.shaped + dt * sdot;
if st.shapeOn
    thetaRef = st.shaped;
else
    thetaRef = st.pref;
end

e = thetaRef - thetaD;
qUnsat = st.Kth * e + st.integ;
if st.shapeOn
    qCmd = cl_clip(qUnsat, -qLim, qLim);
else
    qCmd = qUnsat;
end
aRaw = st.Kq * (qCmd - st.qf);
aMax = mu * st.deltaMax;
aCmd = cl_clip(aRaw, -aMax, aMax);
st.deltaCmd = aCmd / mu;
iDot = st.KiTh * e + st.kaw * (qCmd - qUnsat) ...
    + (st.kaw / max(st.Kq, 1e-9)) * (aCmd - aRaw);
st.integ = st.integ + dt * iDot;

deltaCmd = st.deltaCmd;
info.e = e;
info.thetaRef = thetaRef;
info.qCmd = qCmd;
info.aCmd = aCmd;
info.saturated = abs(aCmd) >= 0.98 * max(aMax, 1e-9) || abs(qCmd - qUnsat) > 1e-8;
end


function [y, buf] = delay_push(buf, u, nDelay)
if nDelay <= 0
    y = u;
    return
end
y = buf(1);
if nDelay == 1
    buf(1) = u;
else
    buf(1:end-1) = buf(2:end);
    buf(end) = u;
end
end


function y = lpf_step(y, u, tau, dt)
a = exp(-dt / max(tau, 1e-9));
y = a * y + (1 - a) * u;
end

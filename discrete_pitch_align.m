function st = discrete_pitch_align(st, theta, q, thetaCmd)
%DISCRETE_PITCH_ALIGN  Start the filters at the current measurement.
%
%   Used when a new stage lights so an empty delay buffer is not read as
%   a real attitude error. The integrator starts at zero.

if st.nDelay > 0
    st.thetaBuf(:) = theta;
    st.qBuf(:) = q;
end
st.pref = thetaCmd;
st.shaped = thetaCmd;
st.qf = q;
st.trim = 0;
st.integ = 0;
st.deltaCmd = 0;
end

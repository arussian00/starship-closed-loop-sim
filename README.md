# Starship closed-loop sim

The pitch controller flies the vehicle. [matlab-rocket-sim](https://github.com/arussian00/matlab-rocket-sim) is the mission (ascent, hot staging, booster catch, ship splashdown). [rocket-controls-simulink](https://github.com/arussian00/rocket-controls-simulink) on `main` is the pitch law (steps 1-2: trajectory gain scheduling with `1/mu`, a command governor, anti-windup, and an inner rate loop). This repo does not copy either tree and does not modify them.

The sister window is [starship-simple-controls-playback](https://github.com/arussian00/starship-simple-controls-playback). That one plays the controller back along a recorded ascent. This one puts the gimbal command inside the integration loop.

## Run in MATLAB Online

From `/MATLAB Drive/Repositories`:

```matlab
cd('/MATLAB Drive/Repositories')
gitclone("https://github.com/arussian00/starship-closed-loop-sim.git")
cd starship-closed-loop-sim
!git fetch origin cursor/closed-loop-pitch-4f8f
!git checkout cursor/closed-loop-pitch-4f8f
addpath(pwd)
rehash
clear functions
run_closed_loop
```

`gitclone` needs R2023b or newer. MATLAB Online is current. The first `run_closed_loop` clones `matlab-rocket-sim` if it is not already under `/MATLAB Drive/Repositories`. The pitch controller is read from `main` in a separate folder, `rocket-controls-simulink-main`. A `rocket-controls-simulink` checkout you already have is not checked out, pulled, or reset.

`!git pull` on MATLAB Drive has bus-errored and left half-written files. Update this branch with fetch and a hard reset, then refresh the function cache. Files written by `!git` stay invisible to the Command Window until `addpath(pwd)` and `rehash`. `clear functions` drops a copy MATLAB already loaded. Call `run_closed_loop`, not `run_closed_loop.m`.

```matlab
cd('/MATLAB Drive/Repositories/starship-closed-loop-sim')
!git fetch origin cursor/closed-loop-pitch-4f8f
!git reset --hard origin/cursor/closed-loop-pitch-4f8f
addpath(pwd)
rehash
clear functions
run_closed_loop
```

If a reset is interrupted, `!git restore --source=HEAD -- .` puts the last good tree back, then run the four cache lines again.

The mission-control window is one figure, tag `starshipClosedLoop`, reused on the next run. It opens maximized (`cfg.figureMode = 'maximized'` in `closed_loop_config.m`) and the panel fonts scale when the window changes size. Set `cfg.figureMode = 'docked'` to dock it. The figure background stays white. Graph interiors stay dark. Axis titles, tick labels, and legends that sit on the white background are near-black and bold. On R2025a and newer the figure `Theme` is set to `'light'`.

Needs **Control System Toolbox** only if you want `design_pitch_controller` to supply the gains. The flight itself uses the same formulas that function uses by default (`Kq = 2`, `Kth = 0.5`, `KiTh = 0.01`), so the ascent runs on core MATLAB. **Simulink** is not required to fly. It is used when you ask for a `sim('boosterPitchTVC')` check and Simulink is installed. If the controller cannot be built, the open-loop mission still plays and the window says `controller unavailable: <reason>`.

## Commands

| Command | What you get |
|---|---|
| `run_closed_loop` | Nominal open-loop flight, then the closed-loop flight, then the animation |
| `run_closed_loop(2)` | Same animation, twice as fast |
| `run_closed_loop(1, false)` | Summaries only |
| `run_closed_loop(1, true, 'gust')` | Design gust from `vehicle_params` (15 m/s at 14 s on main) |
| `run_closed_loop(1, false, 'playback')` | `playback_wind` or `load_relief_wind` when that function and its fields exist, otherwise the design gust |
| `run_closed_loop(1, false, 10)` | Steady 10 m/s wind, positive downrange (same sign as the sim's `P.wind`) |
| `out = run_closed_loop(1, false);` | Histories and `out.summary` |

`'gust'` and `'playback'` use the controls-repo sign: a positive gust reduces angle of attack, applied here as a headwind so dynamic pressure and drag move as well. `'none'` (the default) is still air, so the difference from the nominal trajectory is the controller.

## How the loop is closed

`starshipStack` points thrust with ideal steering and has no attitude state. This run keeps that translational model (same thrust magnitude, throttle bucket, specific impulse, and drag) and adds the pitch states inside the same fixed step:

- Body pitch and pitch rate, nose angle from local vertical.
- Gimbal angle and gimbal rate. The actuator is the second-order servo from `build_pitch_model` (`actHz`, `actZeta`), with the angle and rate stops.
- TVC moment `F * arm * sin(delta)`. Positive `delta` increases pitch, which is the sign in `theta'' = mu*sin(delta) + lambda*theta`.
- Aerodynamic moment `q * CNa * A * lcp * sin(alpha)`, with `lcp = -staticMargin * diameter` from `vehicle_params`.
- Normal force from that same angle of attack, so lift and extra drag change the trajectory. At zero angle of attack and zero gimbal the translational derivative matches `starshipStack`.
- Thrust direction is body attitude plus gimbal.

The controller runs at `cfg.controlHz` (100 Hz). Each period it reads angle of attack and angle-of-attack rate, delayed by the 20 ms sensor delay (2 samples), and holds `delta_cmd` until the next sample. The law is the Simulink cascade, sampled:

1. Command prefilter `1/(cmdTau*s+1)`.
2. Lead limit, then a rate limit (the command governor). The 5 deg design step stays off the floors, same as the model.
3. Attitude loop `Kth` and `KiTh`, inner rate loop `Kq` on the filtered rate.
4. Back-calculation from the rate-command clip and the acceleration clip.
5. `delta_cmd = a_cmd / mu`, with `mu = F*arm/I` recomputed from the current thrust, mass, and length.

On the stack, the pitch command is the sim's steering: vertical, then the 0.8 deg kick, then the flight-path angle (gravity turn). On the ship, the command is the sim's altitude-hold thrust direction. The controller tracks that attitude. `mu` and the aerodynamic moment follow the vehicle, so one set of gains covers liftoff, max-q, and the ship.

Hot staging hands the closed-loop position, velocity, mass, pitch, and pitch rate to `simulateBooster`. The booster flip, boostback, grid fins, and tower catch, and the ship's belly-flop and splashdown, are that function's guidance. The pitch-TVC law does not run those phases: they use RCS, grid fins, or a different attitude loop. The clock and the vehicle HUD say `guidance (controller off)` there. The ascent still moves the catch and the splashdown, because staging is wherever the controller flew.

Steps 3-6 are hooks, used only when the function and the fields exist. A missing piece is a note. Monte Carlo is reported and not launched. A moving-CG schedule replaces arm and CP on the stack only. Grid-fin commands are recorded for the summary; the return still uses the sim's fins.

## What the window shows

One figure, banner **CLOSED LOOP (controller flying)**. Rectangles match the playback layout.

| Row | Panels |
|---|---|
| Scene | Overview, Earth, Super Heavy camera, Starship camera |
| Upper | Altitude, speed, g-load, dynamic pressure |
| Lower | Throttle, AoA, mass, heating |
| Controls | Commanded vs actual pitch, gimbal angle with the stop lines (rate on the same axes), AoA and q\*alpha, tracking error |

Dashed traces are the nominal open-loop trajectory from the same `rocket_starship_sim` numbers, drawn under the closed-loop path.

## Summary

`run_closed_loop` prints max gimbal against the stop, max `|q*alpha|`, peak tracking error, staging time, altitude, speed, and flight-path angle against the nominal ascent, and catch and splashdown miss against this run's open-loop flight and against the published open-loop pair (catch **-0.08 m**, splash **-1.47 m**).

## Headless run (Octave)

Simulink was not installed, so `sim('boosterPitchTVC')` was not executed. The discrete law was compared with the same block diagram integrated at 1 ms. On a 5 deg step the two traces differ by **0.025 deg** at liftoff (`mu = 0.716`) and at max-q (`mu = 0.668`, `lambda = 0.00248`). Final value at the design stop time (22 s) is 5.15 deg, overshoot about 3.5 percent. The slow integrator pole near `-0.021` has not finished settling by 22 s, which is the same weak `Ki` the design uses on purpose.

Rigid-body closed-loop poles at liftoff: `-1.133`, `-0.846`, `-0.021`. With the rate filter and the actuator, the slow poles stay near `-1.72`, `-0.70`, and `-0.021`, and the actuator pair is near `-27.3 ± 26.4j`. Max-q shifts those by less than `0.01`. All of them are in the left half-plane.

With the nose on the guidance attitude and the gimbal at zero, the translational derivative matches `starshipStack` to `0` at liftoff, 30 s, and 80 s. A 20 s closed-loop climb reached 1.01 km, **0.1 m** from the open-loop altitude, with peak angle of attack 0.51 deg and peak gimbal 1.78 deg.

The full mission in still air:

| | Closed loop | Open loop, this run | Published open loop |
|---|---|---|---|
| Hot staging | 144.60 s, 60.16 km, 1577 m/s | 144.57 s, 59.78 km, 1583 m/s | — |
| Catch miss | -0.072 m | -0.083 m | -0.08 m |
| Splashdown miss | -1.463 m | -1.475 m | -1.47 m |

Peak gimbal on the closed-loop ascent was **3.42 deg** (stop 15 deg). Peak `|q*alpha|` was **1.03 kPa·deg**. Peak pitch-tracking error was **3.81 deg**. Through the stack, the 30 s samples sit near zero angle of attack (the 20 s climb peaked at 0.51 deg); the larger error is later, as the ship acquires the altitude-hold attitude. The booster return and the ship belly-flop were the sim's guidance. A positive design gust reduces angle of attack (checked: 0 deg in still air, -4.3 deg with the 15 m/s gust at 200 m/s), which is the sign of `gustAccPerWind`.

`verify_closed_loop` repeats those checks. `verify_closed_loop('core')` stops before the full mission. Octave is only the headless check; the flight code is MATLAB. `sim('boosterPitchTVC')` runs when `sim` and `build_pitch_model` are on the path. A thrown `sim` is caught and the discrete step is kept.

## Files

| File | Role |
|---|---|
| `run_closed_loop.m` | Entry point |
| `setup_paths.m` | Find or clone the sim; read controls `main` from a separate folder |
| `closed_loop_config.m` | URLs, figure tag, controller rate |
| `pitch_law.m` | Gains, from `design_pitch_controller` when it runs |
| `discrete_pitch_init.m`, `discrete_pitch_step.m` | Sampled cascade |
| `ascent_dynamics.m` | Translation, pitch, actuator |
| `fly_closed_loop.m` | Fixed-step ascent, then `simulateBooster` for the returns |
| `nominal_flight.m` | Open-loop reference |
| `closed_loop_wind.m` | Still air, design gust, playback wind, or a steady wind |
| `closed_loop_extensions.m` | Guarded hooks for steps 3-6 |
| `pitch_step_reference.m` | 1 ms diagram and optional `sim('boosterPitchTVC')` |
| `verify_closed_loop.m` | Headless checks |
| `animate_closed_loop.m` | The one mission-control window |
| `present_figure.m`, `closed_loop_theme.m`, `mission_control_layout.m` | Window, colors, panel positions |

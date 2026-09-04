# PX4-style TECS baseline

## Architecture

The Baseline `PX4Autopilot` longitudinal path is:

```text
altitude / calibrated airspeed setpoints
                  |
           Px4TecsController
             /           \
      pitch setpoint   throttle setpoint
           |                 |
   Px4PitchController        engine
           |
        elevator
```

TECS never writes elevator. When TECS is disabled, the configured standalone
Pitch Hold target and manual throttle passthrough remain in use. Enabling TECS
resets its estimator/integrator state and synchronizes its first pitch and
throttle outputs to the current pitch and passthrough throttle. Disabling it
restores the prior Pitch Hold enabled state.

## Units

Control calculations, controller settings, GNC state, and raw telemetry use
SI units exclusively:

| Quantity | Internal/raw unit and reference |
| --- | --- |
| TECS altitude | m AGL |
| TECS calibrated airspeed | m/s CAS |
| Vertical speed | m/s |
| Linear acceleration | m/s^2 |
| Angle | rad |
| Angular rate | rad/s |
| Angular acceleration | rad/s^2 |
| Throttle and control commands | normalized 0..1 |

The runtime `AircraftState` snapshot stores altitude AGL and ASL in metres,
CAS/TAS in metres per second, attitude angles in radians, angular rates in
radians per second, and normalized control commands. The TECS UI defaults to
metres and metres per second. Pitch limits and pitch slew are displayed in
degrees and degrees per second for readability; their conversion occurs in the
GNC panel boundary and the stored `Px4TecsSettings` values remain radians and
radians per second. Capture copies current altitude AGL and calibrated airspeed
directly from that SI snapshot into the target state.

The Viz HUD uses the same runtime snapshot as GNC and telemetry. It displays
`Alt AGL` in metres and CAS/TAS in metres per second. Attitude and course are
converted from radians to degrees only while formatting the HUD, and control
surface/throttle values remain normalized commands. The altitude cue also uses
metres AGL. Viz does not read separate JSBSim feet or knot properties.

All boundary conversion uses the helpers in `common/math/Math.hpp`. The exact
feet-to-metres and knots-to-metres-per-second factors are defined once there;
UI, Viz, scenario/trim adapters, and the JSBSim property adapter do not carry
private conversion constants. The data flow is raw external unit, one boundary
conversion, SI-normalized state/controller/telemetry, and an optional display
conversion. TECS control laws contain no unit conversion.

Trim keeps its aviation-friendly ft/kt/deg editor, but `TrimRequest` stores m,
m/s, and rad and `TrimResult` stores SI values. Conversion to JSBSim's native
initial-condition setters occurs only in `Aircraft::InitializeForTrim`; result
conversion occurs only while rendering the Trim panel.

The version-1 Scenario contract deliberately retains its explicit aviation
input keys for compatibility: `initial_condition.altitude_ft` is altitude
above mean sea level and `initial_condition.airspeed_kts` is calibrated
airspeed. These are JSBSim initial-condition adapter values, not TECS state.
Once running, TECS reads `AltitudeAgl().M()` and
`CalibratedAirspeed().Mps()`. AGL and MSL values are therefore never stored in
the same TECS target or telemetry signal. Display units may differ at a UI
boundary, but control and stored state remain SI.

## PX4 v1.17 behavior represented

The implementation follows the normal fixed-wing cruise path in PX4 v1.17:

- altitude and airspeed errors produce limited vertical-speed and airspeed-rate
  demands;
- specific potential and kinetic energy rates are formed from `g*h_dot` and
  `V*V_dot`;
- total-energy rate error controls throttle using trim feed-forward, damping,
  an integrator, saturation-direction anti-windup, and slew/absolute limits;
- energy-balance rate error controls pitch using equal speed/height weighting
  in normal cruise, damping, feed-forward, an integrator, and pitch slew/angle
  limits;
- the airspeed derivative and total-energy-rate estimate are low-pass filtered;
- underspeed progressively moves pitch weighting to airspeed priority, blocks
  throttle integrator growth, blends throttle toward maximum, and limits
  pitch-up demand;
- overspeed progressively moves pitch weighting to airspeed priority and
  blends throttle toward minimum;
- the altitude reference advances at the configured climb/sink rates instead
  of applying an altitude step directly to the energy controller.

The design was checked against both the official
[PX4 v1.17 TECS source](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/lib/tecs/TECS.cpp),
the [current PX4 TECS source](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tecs/TECS.cpp),
and the [PX4 controller diagrams](https://docs.px4.io/v1.17/en/flight_stack/controller_diagrams).

## Deliberately omitted

This baseline does not reproduce the full PX4 mode machine, estimator,
equivalent-to-true airspeed conversion, landing flare, takeoff/climbout modes,
fast-descend mode, VTOL transition, glider logic, load-factor/turn drag
compensation, terrain following, or mission/navigation behavior.

## C172x tuning

Defaults are held in `Px4TecsParameterMetadata.hpp` and copied into a settings
snapshot, rather than embedded in the control equations. Trim synchronization
replaces `FW_THR_TRIM` with the actual C172x trim result.

| Parameter | Default |
| --- | ---: |
| `FW_P_LIM_MIN` / `FW_P_LIM_MAX` | -10 / +15 deg |
| `FW_THR_MIN` / `FW_THR_MAX` | 0 / 1 |
| `FW_AIRSPD_MIN` / `FW_AIRSPD_MAX` | 25 / 62 m/s CAS |
| `FW_T_CLMB_MAX` / `FW_T_SINK_MAX` | 2.5 / 2.0 m/s |
| altitude / airspeed error gain | 0.10 / 0.40 1/s |
| `FW_T_THR_DAMP` / `FW_T_I_GAIN_THR` | 0.25 / 0.08 |
| `FW_T_PTCH_DAMP` / `FW_T_I_GAIN_PIT` | 0.70 / 0.03 |
| `FW_T_SEB_R_FF` | 1.0 |
| `FW_T_STE_R_TC` | 0.5 s |
| pitch / throttle slew | 8 deg/s / 0.35 1/s |

The deterministic `px4_tecs_integration_tests` executable runs level, climb,
descent, airspeed increase/decrease, combined, and underspeed-protection cases
at the normal 120 Hz simulation rate.

## Telemetry and Monitor

The runtime telemetry registry publishes the complete TECS input, shaped
setpoint, specific-energy, energy-rate, controller-contribution, output, limit,
and protection state under `autopilot/tecs/*`. These channels are searchable in
Monitor's dynamic Add Plot catalog. The `PX4 TECS` preset adds Altitude,
Airspeed, Energy Errors, and Commands plots without introducing a TECS-specific
chart path.

The reference-bearing raw paths are `altitude_agl`, `target_altitude_agl`, and
`internal_altitude_setpoint_agl` in metres, plus `airspeed_cas` and
`target_airspeed_cas` in metres per second. Monitor plots these SI values
directly and obtains their unit labels from the signal catalog.

The version-1 reproducible Scenario and MCAP contract remains intentionally
limited to the existing `roll_hold` experiment. TECS flight validation is kept
in the native deterministic integration test so this implementation does not
silently change that external contract's schema or semantics.

## Pre-tuning C172x validation snapshot

The following results were produced at 120 Hz with an 80 kt trimmed initial
condition. Commands were applied at 5 seconds and each case ran for 70 seconds.
Final errors are ten-second tail-window means. This table preserves the
original baseline recorded before the focused C172x tuning pass.

| Case | Final altitude error | Final airspeed error | Altitude overshoot | Airspeed range | Pitch range | Throttle range | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Level hold | -0.001 m | 0.000 m/s | 0.000 m | 41.152-41.164 m/s | 2.091-2.335 deg | 0.637-0.641 | PASS |
| Altitude +50 m | 0.767 m | 0.020 m/s | 3.818 m | 40.692-41.567 m/s | 0.539-6.865 deg | 0.096-1.000 | PASS |
| Altitude -50 m | -1.249 m | -0.054 m/s | 4.807 m | 40.666-41.814 m/s | -1.758-4.580 deg | 0.000-1.000 | PASS |
| Airspeed +5 m/s | -1.325 m | 0.119 m/s | 0.000 m | 41.148-46.051 m/s | -0.118-2.422 deg | 0.639-1.000 | PASS |
| Airspeed -5 m/s | 1.157 m | -0.174 m/s | 0.000 m | 36.311-41.158 m/s | 2.106-3.738 deg | 0.250-0.639 | PASS |
| Combined +50 m/+3 m/s | 0.217 m | 0.099 m/s | 0.005 m | 41.035-44.216 m/s | 0.544-5.304 deg | 0.301-1.000 | PASS |
| Underspeed protection | 0.486 m | 0.038 m/s | 0.000 m | 42.979-43.755 m/s | 0.935-5.860 deg | 0.336-1.000 | PASS |

Across these cases, the maximum tail pitch-setpoint tracking error was 0.168
deg, elevator saturation was 0%, and sustained tail oscillation stayed below
0.30 deg pitch and 0.01 normalized throttle. The underspeed case activated the
protection path, commanded maximum throttle, and relaxed pitch demand instead
of continuing to increase nose-up demand.

## Focused C172x tuning pass

The integration executable now evaluates the final ten seconds using mean,
maximum-absolute, and RMS altitude/airspeed errors. It also measures settling
time, overshoot, pitch and throttle peak-to-peak motion, pitch-target tracking,
and actuator/command-limit saturation duration. A case is not accepted from a
single final sample.

Three C172x defaults changed:

| Parameter | Before | After | Reason |
| --- | ---: | ---: | --- |
| `FW_T_HRATE_P` | 0.08 1/s | 0.10 1/s | Reduced altitude tail error and both altitude-step overshoots; improved combined settling without sustained hunting. |
| `FW_T_SPDWEIGHT_P` | 0.25 1/s | 0.40 1/s | Brought both +/-5 m/s airspeed steps into the 10-20 second settling goal and reduced tail speed error. |
| `FW_T_THR_DAMP` | 0.35 | 0.25 | Reduced rate-noise response, descent tail throttle/pitch motion, and airspeed-up throttle saturation while retaining settling goals. |

The 0.07 altitude-gain trial was rejected because descent overshoot increased
to 5.261 m. A 0.12 trial reduced error further but raised descent tail throttle
peak-to-peak motion to 0.046, too close to the 0.05 hunting guardrail. Reducing
maximum sink rate from 2.0 to 1.8 m/s was also rejected: descent settling grew
to 42.258 s and tail throttle peak-to-peak motion failed at 0.096. Increasing
throttle damping to 0.45 amplified rate-noise response: descent and
airspeed-up tail throttle peak-to-peak motion failed at 0.122 and 0.124.
Pitch Hold settings and all TECS integral, pitch damping, slew, protection, and
limit parameters were left unchanged.

### Final tuned snapshot

| Case | Tail altitude mean / max / RMS | Tail airspeed mean / max / RMS | Overshoot | Settling | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| Level hold | -0.001 / 0.005 / 0.003 m | 0.000 / 0.003 / 0.002 m/s | 0.000 m | 0.000 s | PASS |
| Altitude +50 m | 0.546 / 0.715 / 0.554 m | 0.011 / 0.018 / 0.011 m/s | 3.598 m | 24.125 s | PASS |
| Altitude -50 m | -1.020 / 1.197 / 1.025 m | -0.035 / 0.048 / 0.036 m/s | 4.307 m | 30.358 s | PASS |
| Airspeed +5 m/s | -1.124 / 1.156 / 1.124 m | 0.071 / 0.082 / 0.071 m/s | 0.000 m | 14.017 s | PASS |
| Airspeed -5 m/s | 0.956 / 0.977 / 0.956 m | -0.106 / 0.119 / 0.106 m/s | 0.000 m | 16.725 s | PASS |
| Combined +50 m/+3 m/s | 0.003 / 0.198 / 0.103 m | 0.058 / 0.070 / 0.058 m/s | 0.156 m | 35.342 s | PASS |
| Underspeed protection | 0.320 / 0.515 / 0.334 m | 0.032 / 0.038 / 0.032 m/s | 0.000 m | 36.167 s | PASS |

| Case | CAS range | Pitch range | Throttle range | Tail pitch / throttle p-p | Overall throttle saturation | Tail actuator saturation |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Level hold | 41.150-41.163 m/s | 2.075-2.342 deg | 0.635-0.642 | 0.234 deg / 0.002 | 0.000 s | 0.000 s |
| Altitude +50 m | 40.706-41.536 m/s | 0.209-7.084 deg | 0.080-1.000 | 0.257 deg / 0.003 | 2.883 s | 0.000 s |
| Altitude -50 m | 40.755-41.808 m/s | -1.913-5.016 deg | 0.000-1.000 | 0.273 deg / 0.011 | 5.958 s | 0.000 s |
| Airspeed +5 m/s | 41.148-46.095 m/s | -0.117-2.452 deg | 0.639-0.971 | 0.304 deg / 0.002 | 0.000 s | 0.000 s |
| Airspeed -5 m/s | 36.249-41.158 m/s | 2.085-3.737 deg | 0.267-0.659 | 0.186 deg / 0.002 | 0.000 s | 0.000 s |
| Combined +50 m/+3 m/s | 41.036-44.237 m/s | 0.293-5.338 deg | 0.311-1.000 | 0.302 deg / 0.003 | 3.092 s | 0.000 s |
| Underspeed protection | 41.148-43.607 m/s | -0.044-6.121 deg | 0.495-1.000 | 0.285 deg / 0.002 | 2.800 s | 0.000 s |

The underspeed case raises the configured minimum-speed boundary above current
speed at the same instant as the +50 m altitude request. Protection activated
on the first post-command update (5.008 s), reached 1.000 throttle, and relaxed
the pitch target by 2.794 deg. Minimum CAS remained 41.148 m/s and the aircraft
did not continue toward stall.

For targeted sensitivity runs, select a case and override existing settings,
for example:

```powershell
build/px4_tecs_integration_tests --scenario airspeed-up --airspeed-gain 0.40
```

Run the focused validation with:

```powershell
cmake --build build --target px4_tecs_controller_tests px4_tecs_integration_tests
ctest --test-dir build -R px4_tecs --output-on-failure
```

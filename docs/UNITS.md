# Unit and Physical Quantity Policy

## Internal convention

Simulation-domain state, controller inputs and outputs, stored settings, and
raw telemetry use SI units:

| Quantity | Internal unit | Semantic rule |
| --- | --- | --- |
| Length and altitude | m | Reference is named explicitly as AGL or ASL/MSL |
| Airspeed and velocity | m/s | CAS and TAS are separate named values |
| Acceleration | m/s^2 | Body-axis components retain their frame name |
| Angle | rad | UI presentation may use degrees |
| Angular rate | rad/s | UI presentation may use degrees per second |
| Angular acceleration | rad/s^2 | |
| Time | s | |
| Throttle | normalized 0..1 | Other controls retain their documented normalized range |
| Temperature | K | FDM environment snapshots are SI |
| Pressure | Pa | FDM environment snapshots are SI |

TECS uses altitude above ground level in metres and calibrated airspeed in
metres per second. `InitialCondition` uses altitude above sea level in metres,
and the reference is encoded as `altitudeAslM`; these values are not
interchangeable.

## Boundaries

Values are converted once at a system boundary:

1. `sim/jsbsim/Properties` and `FDMStateAccess` normalize JSBSim's native
   feet, feet-per-second, Rankine, and pounds-per-square-foot values.
2. `Aircraft` converts SI initial and trim conditions when calling JSBSim's
   native initial-condition API.
3. `SimulationScenarioSerializer` preserves the version-1 YAML contract
   (`*_deg`, `altitude_ft`, and `airspeed_kts`) while storing the parsed domain
   object in SI.
4. Trim UI may display aviation units; it converts through `common/math/Math`
   when committing to the SI `TrimRequest`.
5. UI angles may be displayed in degrees, but GUI models and messages retain
   SI domain values. Monitor, Viz, and GNC read the same SI snapshots and
   telemetry.

All shared conversion constants and helpers live in `common/math/Math.hpp`.
Controller equations must not perform display- or JSBSim-unit conversion.

## Display convention

The default GNC, Monitor, Viz, Simulation, and Scenario displays use metres and
metres per second. Human-facing attitude, heading, and course remain degrees,
converted only while rendering or accepting UI input. Labels include both the
unit and semantic reference, for example `Altitude AGL [m]`, `Altitude ASL
(m)`, `CAS [m/s]`, and `TAS [m/s]`.

An aviation display mode may later show feet and knots, but it must remain a
display-only transformation; it must not change configuration, controller,
simulation, or telemetry storage.

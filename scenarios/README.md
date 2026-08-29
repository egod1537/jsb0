# Reproducible scenarios

Each YAML file in this directory is a complete JSB0 execution definition and
is validated against `contract/scenario/scenario.schema.json` before the first
simulation tick. Aircraft, autopilot, initial condition, timestep, duration,
trim/environment choice, and ordered commands come only from the file.

Run either autopilot with the same built executable:

```text
jsb0-runner --scenario scenarios/roll_hold_primary.yaml --output run-primary
jsb0-runner --scenario scenarios/roll_hold_baseline.yaml --output run-baseline
```

The reproducibility identity is the immutable JSB0 commit SHA plus the
scenario bytes (recorded as a SHA-256 digest). CLI flags do not override
scenario execution semantics.

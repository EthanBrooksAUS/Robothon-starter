# Judge Brief — Orbital Bayonet Repair

Inspect these first:

1. `artifacts/demo.mp4` for the complete autonomous task and live evidence bars.
2. `artifacts/report.json` for MuJoCo-derived success conditions.
3. `artifacts/evaluation.json` for the 32-rollout residual ablation.
4. `artifacts/trajectory.json` for per-frame observations, actions, faults, residuals, contacts, and confidence.
5. `orbital_bayonet_scene.xml` for the five-finger hand, physics, sensors, and actuators.

The deliberately injected visual bias is recorded in `fault_injection_xyz`; the correcting action is recorded independently in `residual_action_xyz`. A proof-load wrench above 8 N is applied to the plug during `tug_test`. This makes closed-loop behavior falsifiable rather than merely described.

The project is intentionally compact: one MJCF scene, one runner, one validator, no downloaded assets. See `rubric_scorecard.json` for direct claim-to-file mapping and `README.md` for limitations.


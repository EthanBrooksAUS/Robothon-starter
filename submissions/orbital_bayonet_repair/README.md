# Orbital Bayonet Repair

Orbital Bayonet Repair is a self-contained MuJoCo dexterous-manipulation task for Robothon 2026. In microgravity, a five-finger service hand repairs a damaged satellite power connector: it grasps a keyed plug, rejects a reproducible camera-calibration fault, performs compliant insertion, rotates a bayonet collar, survives a proof-load tug, presses a guarded verification channel, and exports an auditable trajectory dataset.

## One-command run

From the repository root:

```bash
python3 -m pip install -r requirements.txt
python submissions/orbital_bayonet_repair/run_orbital_repair.py
```

This generates a 60-second, 960x544 demo plus all JSON evidence in `submissions/orbital_bayonet_repair/artifacts/`. For a fast end-to-end check:

```bash
python submissions/orbital_bayonet_repair/run_orbital_repair.py --quick
python submissions/orbital_bayonet_repair/validate_submission.py
```

The runner selects EGL on headless Linux. If no OpenGL context exists, it writes a deterministic schematic video driven by the same MuJoCo states instead of failing.

## Task and success criteria

The controller must complete all of these measurable conditions:

1. Reduce keyed plug/socket error below 12 mm.
2. Rotate the bayonet collar beyond 1.20 rad.
3. Apply and recover from a proof load above 8 N.
4. Depress the verification channel beyond 25 mm.
5. Demonstrate closed-loop corrections and finish above 0.85 policy confidence.

The high-level nine-stage plan is deterministic for reproducibility. Its residual controller is closed-loop: it reads plug/socket frame sensors, plug velocity, all five fingertip sensors, collar state, and verification state; then adjusts gantry position and grasp closure. The demo deliberately injects calibration bias and a proof-load impulse so recovery is visible and machine-checkable.

## MuJoCo platform

- 16-channel hand: gantry XYZ, wrist yaw/pitch/roll, and two independently actuated joints on each of five fingers.
- Two task actuators: locking collar and verification channel.
- 13 sensors: frame position/velocity, five fingertip touch sensors, collar position/velocity, button position/contact.
- Free, slide, and hinge joints; frictional collision geometry; equality constraints; implicit integration; and external wrench injection.
- Procedural MJCF only—no meshes, checkpoints, downloads, or hidden services.

## Evidence artifacts

Running the demo creates:

| Artifact | Evidence |
|---|---|
| `artifacts/demo.mp4` | Startup, nine task phases, robot motion, outcome, and four live signal bars |
| `artifacts/report.json` | Physics inventory, pass/fail conditions, raw/corrected error, proof load |
| `artifacts/trajectory.json` | Per-frame states, actions, residuals, contacts, confidence, and labels |
| `artifacts/evaluation.json` | 32 seeded disturbance rollouts and stage-prior ablation |
| `artifacts/policy_card.json` | Observation/action spaces, controller scope, recovery cases |
| `artifacts/contact_timeline.json` | Five-finger force and proof-load timeline |
| `artifacts/narration.srt` | Accessible narration aligned to every task phase |

The top ribbon in the video shows all nine phases and progress. Four bottom bars show controller confidence, active-finger fraction, collar lock, and power verification. This avoids depending on font availability in headless judge containers.

## Rubric mapping

| Criterion | Concrete coverage |
|---|---|
| Runnability | One command, deterministic seed, no external assets, EGL plus fallback, structural validator |
| MuJoCo depth | Custom MJCF, 18 actuators, 13 sensors, 17 robot joints, contacts, equality constraints, external force |
| Task design | Clear safety-critical repair with insertion, locking, proof testing, and conditional verification |
| Control | Hybrid task plan, visual-servo residual, contact balancing, disturbance rejection, fixed-seed ablation |
| Dexterity | Five fingers, ten finger joints, preshape, coordinated closure, fine keyed alignment, collar rotation |
| Engineering | Typed single-purpose runner, named MJCF elements, CLI, structured artifacts, validation, honest scope |
| Presentation | Generated one-minute video, phase ribbon, live metrics, SRT narration, judge brief |
| Innovation | Microgravity connector servicing combines dexterity, fault recovery, proof testing, and dataset export |

## Honest limitations

- The stage planner is authored, not learned; the residual feedback is the reactive part.
- The plug is initially held by a weld constraint so the submission focuses on fine alignment, locking, and verification rather than free-floating capture.
- The 32-rollout ablation is a deterministic dynamics isolation of the residual law. MuJoCo-derived task measurements are reported separately in `report.json`.

## Registration

Replace the placeholder in `registration.json` with the UUID issued by `robothon.ff.com`, add the same UUID to `PR_DESCRIPTION.md`, and update the participant name. The validator intentionally rejects placeholders.


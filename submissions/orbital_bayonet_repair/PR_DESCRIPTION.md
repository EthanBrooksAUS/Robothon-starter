## 🤖 Orbital Bayonet Repair

**Registration UUID:** `d4dc8416-cc0e-4ebb-a255-2372847c3853`

### Project summary

Orbital Bayonet Repair is a self-contained MuJoCo dexterous-manipulation task for satellite servicing in microgravity. A 16-channel, five-finger service hand restores a keyed power connector through sensor self-check, approach, contact-balanced grasping, visual-servo fault recovery, compliant insertion, bayonet locking, an external proof-load test, and guarded power verification.

### Key contributions

- **Closed-loop fault recovery:** frame-sensor feedback rejects an injected camera-calibration bias with bounded residual corrections.
- **Five-finger manipulation:** ten independently actuated finger joints and five touch sensors support contact-balanced grasping.
- **Safety-gated repair:** power verification occurs only after keyed insertion, collar locking, and an 8 N proof-load test.
- **Auditable evaluation:** the runner exports a video, trajectory, contact timeline, runtime report, policy card, captions, and a 32-rollout fixed-seed controller ablation.
- **Self-contained MuJoCo model:** 18 actuators, 13 sensors, frictional contacts, equality constraints, external wrench injection, and no external meshes or checkpoints.

### Results

- Completed all **9/9 task stages** and passed all **6/6 terminal conditions**.
- Reduced median servo error from **8.615 mm to 1.551 mm** (**82.0%**).
- Reached a minimum insertion error of **4.042 mm**.
- Sustained a measured peak proof load of **8.9987 N**.
- Achieved **32/32 successful closed-loop rollouts**; the stage-prior-only ablation achieved 0/32.

### Run and validate

From the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r submissions/orbital_bayonet_repair/requirements.txt
.venv/bin/python submissions/orbital_bayonet_repair/run_orbital_repair.py
.venv/bin/python submissions/orbital_bayonet_repair/validate_submission.py
```

Use `--quick` with the runner for a faster smoke test. The default run generates a 60-second demonstration and structured evidence under `submissions/orbital_bayonet_repair/artifacts/`.

### Main files

- `run_orbital_repair.py` — controller, simulation, rendering, and evidence generation
- `orbital_bayonet_scene.xml` — procedural five-finger robot and satellite connector scene
- `validate_submission.py` — structural, metadata, artifact, and MuJoCo checks
- `evaluation_report.json` — rubric-aligned evaluation summary
- `artifacts/report.json` — measured runtime results and terminal conditions
- `artifacts/evaluation.json` — fixed-seed residual-controller ablation
- `artifacts/demo.mp4` — generated task demonstration

### Scope and limitations

The nine-stage task schedule is authored and deterministic; the visual/contact residual controller is the reactive component. The connector begins constrained in-hand so the task focuses on fine alignment, locking, proof testing, and verification. The fixed-seed ablation isolates residual-controller dynamics, while MuJoCo-derived task measurements are reported separately in `artifacts/report.json`.
